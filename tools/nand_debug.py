import struct, sys
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
p = r'd:\project\Moral_I9_Simulator\bin\Rom\moral-i9.bin'
d = open(p,'rb').read()
PAGE_SIZE=2112; DATA_SIZE=2048; BPS=512; PPB=64

# Investigate block 468 (contains page 29993 with MB_MSTAR_WQVGA .)
blk = 468
blk_page0 = blk * PPB
print(f'Block {blk}: pages {blk_page0}..{blk_page0+PPB-1}')
for i in range(0, PPB, 4):
    pg = blk_page0 + i
    sp = d[pg*PAGE_SIZE+DATA_SIZE:pg*PAGE_SIZE+DATA_SIZE+8]
    lbn = struct.unpack_from('>H', sp, 1)[0]
    pib = sp[4]
    sys.stdout.write(f'  page {pg} (blk_off={i}): spare={sp.hex()} LBN={lbn} pib={pib}\n')

# Now: what's at page 29993 exactly?
pg = 29993
print(f'\nPage {pg} data (first 128 bytes):')
print(d[pg*PAGE_SIZE:pg*PAGE_SIZE+128].hex())
# And surrounding pages for context
for pg in [29992, 29993, 29994]:
    off = pg*PAGE_SIZE+3*512  # sip=3 = last sector in page
    e = d[off:off+32]
    name = e[0:11].decode('ascii','replace')
    attr = e[11]
    clus = (struct.unpack_from('<H',e,20)[0]<<16)|struct.unpack_from('<H',e,26)[0]
    sz = struct.unpack_from('<I',e,28)[0]
    sys.stdout.write(f'pg {pg} sip=3: [{name}] attr={attr} clus={clus} sz={sz}\n')
