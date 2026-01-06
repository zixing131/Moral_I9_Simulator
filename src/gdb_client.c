#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <pthread.h>

#define SERVER_PORT 6688
#define BUFFER_SIZE 4096
#define MAX_CLIENTS 1

/* 模拟目标系统状态 */
typedef struct
{
    unsigned int registers[32];    /* 32个通用寄存器 */
    unsigned char memory[0x10000]; /* 64KB内存 */
    unsigned int pc;               /* 程序计数器 */
    int running;                   /* 运行状态 */
    unsigned int breakpoints[16];  /* 断点表 */
    int num_breakpoints;           /* 断点数量 */
    int simulate_pc_count;
} TargetSystem;

/* GDB客户端会话 */
typedef struct
{
    int socket;              /* 客户端套接字 */
    pthread_t thread;        /* 处理线程 */
    TargetSystem *gdbTarget; /* 目标系统引用 */
    int active;              /* 会话活跃标志 */
} GDBClient;

/* 全局变量 */
TargetSystem gdbTarget;
GDBClient clients[MAX_CLIENTS];
int activeClientsCount;
int server_socket = -1;
pthread_mutex_t target_mutex = PTHREAD_MUTEX_INITIALIZER;

/* 函数声明 */
void init_target_system(TargetSystem *gdbTarget);
void handle_gdb_command(GDBClient *client, const char *command);
void send_gdb_response(GDBClient *client, const char *response);
void *client_thread(void *arg);
int start_server(int port);
void cleanup(void);
void gdb_server_main();

typedef void (*readAllCpuReg)(int *);
typedef void (*gdb_writeReg)(unsigned int reg, unsigned int value);

typedef void (*gdb_readMem)(unsigned int addr, unsigned int length, void *buffer);
typedef void (*gdb_writeMem)(unsigned int addr, unsigned char value);

readAllCpuReg readAllCpuRegFunc;
gdb_readMem gdb_readMemFunc;
gdb_writeMem gdb_writeMemFunc;
gdb_writeReg gdb_writeRegFunc;

/* 初始化目标系统状态 */
void init_target_system(TargetSystem *gdbTarget)
{
    memset(gdbTarget, 0, sizeof(TargetSystem));
    gdbTarget->pc = 0x1000; /* 设置初始PC值 */
    gdbTarget->num_breakpoints = 0;
}

/* 解析并处理GDB命令 */
void handle_gdb_command(GDBClient *client, const char *command)
{
    pthread_mutex_lock(&target_mutex);
    if (strcmp(command, "?") == 0)
    {
        send_gdb_response(client, "S05");
    }
    else if (strncmp(command, "g", 1) == 0)
    {
        /* 获取所有寄存器 */
        char response[BUFFER_SIZE];
        int offset = 0;
        unsigned int value;
        if (readAllCpuRegFunc != NULL)
            readAllCpuRegFunc(gdbTarget.registers);
        for (int i = 0; i < 17; i++)
        {
            // 转换端序
            value = gdbTarget.registers[i];
            // value = (value >> 24) | ((value >> 8) & 0xff00) | ((value << 8) & 0xff0000) | (value << 24);
            for (int b = 0; b < 4; b++) // 4字节小端输出
            {
                offset += sprintf(response + offset, "%02x", (value >> (8 * b)) & 0xFF);
            }
        }

        send_gdb_response(client, response);
    }
    else if (strncmp(command, "m", 1) == 0)
    {
        /* 读取内存 */
        unsigned int addr, length;
        if (sscanf(command, "m%x,%x", &addr, &length) == 2)
        {

            if (gdb_readMemFunc != NULL)
                gdb_readMemFunc(addr, length, gdbTarget.memory);
            char response[BUFFER_SIZE * 2];
            char *ptr = response;

            for (unsigned int i = 0; i < length; i++)
            {
                ptr += sprintf(ptr, "%02x", gdbTarget.memory[i]);
            }

            send_gdb_response(client, response);
        }
        else
        {
            send_gdb_response(client, "E01"); /* 格式错误 */
        }
    }
    else if (strncmp(command, "M", 1) == 0) // 修改内存
    {
        unsigned int addr, value;
        if (sscanf(command, "M%x,1:%x", &addr, &value) == 2)
        {
            if (gdb_writeMemFunc != NULL)
                gdb_writeMemFunc(addr, value);

            send_gdb_response(client, "OK");
        }
    }
    else if (strncmp(command, "P", 1) == 0)
    {
        unsigned int addr, value, valueBig;
        if (sscanf(command, "P%d=%x", &addr, &value) == 2)
        {
            valueBig = value << 24 | ((value & 0x0000ff00) << 8) | ((value & 0x00ff0000) >> 8) | value >> 24;
            if (gdb_writeRegFunc != NULL)
                gdb_writeRegFunc(addr, valueBig);
            send_gdb_response(client, "OK");
        }
    }
    else if (strcmp(command, "s") == 0 || strcmp(command, "interrupt") == 0)
    {
        gdbTarget.simulate_pc_count = 1;
        gdbTarget.running = 1;
    }

    else if (strcmp(command, "c") == 0)
    {
        gdbTarget.running = 1;
        // printf("continue\n");
        send_gdb_response(client, "");
    }
    else if (strncmp(command, "Z0", 2) == 0)
    {
        /* 设置断点 */
        unsigned int addr;
        if (sscanf(command, "Z0,%x", &addr) == 1)
        {
            if (gdbTarget.num_breakpoints < 16)
            {
                gdbTarget.breakpoints[gdbTarget.num_breakpoints++] = addr;
                // printf("add breakpoint point: %x\n", addr);
                send_gdb_response(client, "OK");
            }
            else
            {
                send_gdb_response(client, "E03"); /* 断点数量超限 */
            }
        }
        else
        {
            send_gdb_response(client, "E01"); /* 格式错误 */
        }
    }
    else if (strncmp(command, "z0", 2) == 0)
    {
        /* 删除断点 */
        unsigned int addr;
        if (sscanf(command, "z0,%x", &addr) == 1)
        {
            for (int i = 0; i < gdbTarget.num_breakpoints; i++)
            {
                if (gdbTarget.breakpoints[i] == addr)
                {
                    /* 移除断点 */
                    for (int j = i; j < gdbTarget.num_breakpoints - 1; j++)
                    {
                        gdbTarget.breakpoints[j] = gdbTarget.breakpoints[j + 1];
                    }
                    gdbTarget.num_breakpoints--;
                    // printf("breakpoint count change: %d\n", gdbTarget.num_breakpoints);
                    send_gdb_response(client, "OK");
                    pthread_mutex_unlock(&target_mutex);
                    return;
                }
            }
            send_gdb_response(client, "E04"); /* 断点未找到 */
        }
        else
        {
            send_gdb_response(client, "E01"); /* 格式错误 */
        }
    }
    else
    {

        // printf("Received unhandled command: %s\n", command);
        /* 未知命令 */
        send_gdb_response(client, "E01");
    }

    pthread_mutex_unlock(&target_mutex);
}

/* 发送GDB响应 */
void send_gdb_response(GDBClient *client, const char *response)
{
    char buffer[BUFFER_SIZE];
    int checksum = 0;
    if (client->active == 0)
    {
        printf("send gdb rsp fail: no active\n");
        return;
    }
    /* 计算校验和 */
    for (const char *p = response; *p; p++)
    {
        checksum += *p;
    }
    checksum &= 0xFF;

    /* 构建响应包 */
    sprintf(buffer, "$%s#%02x", response, checksum);

    /* 发送响应 */
    send(client->socket, buffer, strlen(buffer), 0);
}

/* 客户端处理线程 */
void *client_thread(void *arg)
{
    GDBClient *client = (GDBClient *)arg;
    char buffer[BUFFER_SIZE];
    char command[BUFFER_SIZE];
    int buffer_pos = 0;
    int in_packet = 0;
    int packet_start = 0;
    int packet_len = 0;
    unsigned char checksum = 0;
    unsigned char received_checksum = 0;

    printf("Client connected\n");

    while (client->active)
    {
        /* 接收数据 */
        int bytes_received = recv(client->socket, buffer + buffer_pos,
                                  BUFFER_SIZE - buffer_pos - 1, 0);

        if (bytes_received <= 0)
        {
            /* 客户端断开连接 */
            break;
        }

        buffer_pos += bytes_received;
        buffer[buffer_pos] = '\0';

        /* 解析GDB包 */
        for (int i = 0; i < buffer_pos; i++)
        {
            char tmp = buffer[i];
            if (tmp == '$')
            {
                in_packet = 1;
                packet_start = i + 1;
                packet_len = 0;
                checksum = 0;
            }
            else if (tmp == 0x03)
            {
                // GDB 请求中断
                handle_gdb_command(client, "interrupt");
                // 移除这个字节
                memmove(buffer + i, buffer + i + 1, buffer_pos - i - 1);
                buffer_pos--;
                i--;
            }
            else if (in_packet && tmp == '#')
            {
                in_packet = 0;

                /* 提取接收到的校验和 */
                if (i + 2 < buffer_pos)
                {
                    sscanf(buffer + i + 1, "%02hhx", &received_checksum);

                    /* 验证校验和 */
                    if (checksum == received_checksum)
                    {
                        /* 发送确认 */
                        send(client->socket, "+", 1, 0);

                        /* 提取命令 */
                        memcpy(command, buffer + packet_start, packet_len);
                        command[packet_len] = '\0';

                        /* 处理命令 */
                        handle_gdb_command(client, command);
                    }
                    else
                    {
                        /* 校验和错误 */
                        send(client->socket, "-", 1, 0);
                    }

                    /* 移除已处理的数据 */
                    memmove(buffer, buffer + i + 3, buffer_pos - (i + 3) + 1);
                    buffer_pos -= (i + 3);
                    i = -1; /* 重新开始解析 */
                }
            }
            else if (in_packet)
            {
                checksum += tmp;
                packet_len++;
            }
        }
    }

    /* 清理资源 */
    close(client->socket);
    client->active = 0;
    printf("Client disconnected\n");

    return NULL;
}

/* 启动GDB服务器 */
int start_server(int port)
{
    struct sockaddr_in server_addr;
    int iResult;
    WSADATA wsaData;

    // 初始化Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0)
    {
        printf("WSAStartup failed: %d\n", iResult);
        return 1;
    }

    /* 创建套接字 */
    server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket < 0)
    {
        printf("socket creation failed with error: %d\n", server_socket);
        return -1;
    }

    /* 设置套接字选项 */
    int opt = 1;
    if (setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)))
    {
        printf("setsockopt failed");
        close(server_socket);
        return -1;
    }

    /* 准备服务器地址结构 */
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    /* 绑定套接字 */
    if (bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        printf("bind failed");
        close(server_socket);
        return -1;
    }

    /* 监听连接 */
    if (listen(server_socket, 5) < 0)
    {
        printf("listen failed");
        close(server_socket);
        return -1;
    }

    printf("GDB Server listening on port %d\n", port);
    return 0;
}

/* 清理资源 */
void cleanup(void)
{
    /* 关闭所有客户端连接 */
    for (int i = 0; i < MAX_CLIENTS; i++)
    {
        if (clients[i].active)
        {
            clients[i].active = 0;
            close(clients[i].socket);
            pthread_join(clients[i].thread, NULL);
        }
    }
    activeClientsCount = 0;
    /* 关闭服务器套接字 */
    if (server_socket != -1)
    {
        close(server_socket);
    }

    pthread_mutex_destroy(&target_mutex);
}

/* 主函数 */
void gdb_server_main(void)
{
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    int client_socket;

    /* 初始化目标系统 */
    init_target_system(&gdbTarget);

    /* 启动服务器 */
    if (start_server(SERVER_PORT) < 0)
    {
        return 1;
    }

    /* 初始化客户端数组 */
    memset(clients, 0, sizeof(clients));

    /* 信号处理和清理 */
    atexit(cleanup);

    /* 接受客户端连接 */
    while (1)
    {
        /* 查找可用的客户端槽 */
        int client_idx = -1;
        for (int i = 0; i < MAX_CLIENTS; i++)
        {
            if (!clients[i].active)
            {
                client_idx = i;
                break;
            }
        }

        if (client_idx == -1)
        {
            /* 没有可用槽位，等待一段时间 */
            sleep(1);
            continue;
        }

        /* 接受新连接 */
        client_socket = accept(server_socket, (struct sockaddr *)&client_addr, &client_addr_len);
        if (client_socket < 0)
        {
            printf("accept failed");
            continue;
        }

        /* 初始化客户端结构 */
        clients[client_idx].socket = client_socket;
        clients[client_idx].gdbTarget = &gdbTarget;
        clients[client_idx].active = 1;
        activeClientsCount++;
        /* 创建处理线程 */
        if (pthread_create(&clients[client_idx].thread, NULL, client_thread, &clients[client_idx]) != 0)
        {
            printf("thread creation failed");
            close(client_socket);
            clients[client_idx].active = 0;
            activeClientsCount--;
        }
        else
        {
            /* 设置线程为分离状态，自动回收资源 */
            pthread_detach(clients[client_idx].thread);
        }
    }
}