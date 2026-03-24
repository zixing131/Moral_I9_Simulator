"""
MTK NAND FAT16 scanner for moral-i9.bin.

NAND: 2048-byte data + 64-byte spare per page, 64 pages/block.
Spare: [0]=0xFF(good), [1:3]=LBN(BE), [3]=0x00, [4]=lpib_or_0xFF.

FTL mapping is NOT a simple BPB_LBN + log_sec/256 linear scheme.
The true cluster-to-physical mapping is discovered by physically scanning
ALL 2048-byte pages in NAND data sections and looking for FAT directory sector signatures.

Strategy:
1. Read FAT0 (sectors rsv..rsv+spf16) from a KNOWN working LBN base (BPB_LBN=3365).
2. For each cluster in the FAT chain, compute its expected sector,
   but ALSO physically locate the cluster by scanning for the directory content.
3. Build cluster_to_phys_base[cluster] = physical byte offset of the cluster's first sector.
   We do this by physically scanning all '.' dir-entry starts in NAND and correlating
   with the . entry's own cluster field.

For FILE clusters (non-directory), we use a SEQUENTIAL physical scan relative to the known
directory cluster position, walking by sector size within the physical NAND.

Actually the simpler robust approach: since we know BPB_LBN=3365 works for FAT0 and the root dir,
and the problem is deeper LBN mismatches, we build a complete physical scan that finds
every FAT directory page by signature, then reconstruct the directory tree purely from
physical data rather than relying on logical sector arithmetic.
"""
import struct, sys, os

IMG_PATH = r'd:\project\Moral_I9_Simulator\bin\Rom\moral-i9.bin'
OUT_DIR  = r'd:\project\Moral_I9_Simulator\bin\Rom\CARD\.system\MB_MSTAR_WQVGA'

PAGE_SIZE    = 2112
DATA_SIZE    = 2048
BPS          = 512
PPB          = 64
SECS_PER_PAGE = DATA_SIZE // BPS   # 4
SECS_PER_LBN  = PPB * SECS_PER_PAGE  # 256
BPB_LBN      = 3365

sys.stdout.reconfigure(encoding='utf-8', errors='replace')


def main():
    with open(IMG_PATH, 'rb') as f:
        d = f.read()
    total_pages = len(d) // PAGE_SIZE
    total_blocks = total_pages // PPB
    print(f'[nand_fat] Image {len(d)} bytes, {total_pages} pages')

    # ------------------------------------------------------------------ #
    # Phase 1: build Type-A (LBN, lpib) -> phys page map                  #
    #          and LBN -> physical block map for Type-B fallback            #
    # ------------------------------------------------------------------ #
    lbn_pib_to_page = {}   # Type-A: (lbn, lpib) -> first physical page
    lbn_to_block    = {}   # any type: lbn -> first physical block

    for pg in range(total_pages):
        sp = d[pg * PAGE_SIZE + DATA_SIZE : pg * PAGE_SIZE + DATA_SIZE + 8]
        if sp[0] != 0xFF or sp[3] != 0x00:
            continue
        lbn = struct.unpack_from('>H', sp, 1)[0]
        if lbn >= 0xFFF0:
            continue
        lpib = sp[4]
        blk  = pg // PPB
        if lbn not in lbn_to_block:
            lbn_to_block[lbn] = blk
        if lpib != 0xFF:
            key = (lbn, lpib)
            if key not in lbn_pib_to_page:
                lbn_pib_to_page[key] = pg

    # ------------------------------------------------------------------ #
    # Phase 2: BPB-based linear read for FAT table and root dir            #
    # ------------------------------------------------------------------ #
    def linear_read_sec(log_sec):
        lbn    = BPB_LBN + log_sec // SECS_PER_LBN
        sin    = log_sec % SECS_PER_LBN
        lpib   = sin // SECS_PER_PAGE
        sip    = sin % SECS_PER_PAGE
        phys = lbn_pib_to_page.get((lbn, lpib))
        if phys is None:
            blk = lbn_to_block.get(lbn)
            if blk is not None:
                # Type-B: physical page = blk*PPB + lpib
                phys = blk * PPB + lpib
        if phys is None:
            return b'\xFF' * BPS
        off = phys * PAGE_SIZE + sip * BPS
        seg = d[off : off + BPS]
        return seg + b'\xFF' * (BPS - len(seg)) if len(seg) < BPS else seg

    def linear_read_secs(start, count):
        return b''.join(linear_read_sec(start + i) for i in range(count))

    vbr = linear_read_sec(0)
    assert vbr[3:11] == b'MSDOS5.0', f'VBR mismatch: {vbr[3:11]}'
    bps      = struct.unpack_from('<H', vbr, 11)[0]
    spc      = vbr[13]
    rsv      = struct.unpack_from('<H', vbr, 14)[0]
    nf       = vbr[16]
    root_cnt = struct.unpack_from('<H', vbr, 17)[0]
    spf16    = struct.unpack_from('<H', vbr, 22)[0]
    assert bps == BPS
    root_sec       = rsv + nf * spf16
    first_data_sec = root_sec + (root_cnt * 32 + BPS - 1) // BPS
    print(f'[nand_fat] FAT16: spc={spc} rsv={rsv} nf={nf} root_cnt={root_cnt} spf={spf16}')
    print(f'[nand_fat] root_sec={root_sec} fds={first_data_sec}')

    fat0 = linear_read_secs(rsv, spf16)

    def fat_entry(c):
        return struct.unpack_from('<H', fat0, c * 2)[0]

    def cluster_chain(start):
        chain, seen = [], set()
        c = start
        while 2 <= c < 0xFFF8:
            if c in seen:
                break
            seen.add(c)
            chain.append(c)
            c = fat_entry(c)
            if len(chain) > 500_000:
                break
        return chain

    # ------------------------------------------------------------------ #
    # Phase 3: physical scan to build cluster -> physical base offset map  #
    # ------------------------------------------------------------------ #
    # Scan every 512-byte aligned position in every page's data area       #
    # for valid FAT directory sector content ('. ' 8.3 entry at offset 0)  #
    # Record: cluster_phys_base[cluster] = byte offset in d of the first   #
    # sector of that cluster (where '. ' appears).                          #
    # ------------------------------------------------------------------ #
    print('[nand_fat] Physical scanning for cluster data locations ...')
    cluster_phys_base = {}   # cluster -> physical byte offset of cluster's first sector

    DOT_11 = b'.          '   # 11 bytes

    for pg in range(total_pages):
        pg_data_base = pg * PAGE_SIZE
        # Scan the 4 sectors within this page's data area
        for sip in range(SECS_PER_PAGE):
            off = pg_data_base + sip * BPS
            seg = d[off : off + 32]
            if len(seg) < 32:
                continue
            if seg[0:11] != DOT_11:
                continue
            attr = seg[11]
            if attr not in (0x10, 0x30):  # directory attribute
                continue
            clus = (struct.unpack_from('<H', seg, 20)[0] << 16) | struct.unpack_from('<H', seg, 26)[0]
            if clus < 2 or clus > 100_000:
                continue
            if clus not in cluster_phys_base:
                # Record the physical byte offset of this cluster's first sector
                # The '. ' entry is the first entry of the cluster's first sector
                # if sip==0 this is already page start; if sip>0 we back up
                cluster_phys_base[clus] = off  # absolute byte offset of this sector

    print(f'[nand_fat] Found physical locations for {len(cluster_phys_base)} clusters')

    # ------------------------------------------------------------------ #
    # Phase 4: read cluster data using physical location                   #
    # ------------------------------------------------------------------ #
    def read_cluster_data(c):
        """
        Read all spc sectors of cluster c.
        For each sector offset within the cluster, we try:
          1. cluster_phys_base (direct physical scan result) + sector_offset
          2. linear FAT addressing fallback
        The sectors are laid out in the NAND:
          - Within a physical page: 4 consecutive 512-byte sectors.
          - Across pages: consecutive pages within the same LBN block.
        We compute the sector offset within the cluster and look up accordingly.
        """
        base_phys = cluster_phys_base.get(c)
        parts = []
        for s in range(spc):
            if base_phys is not None:
                # Sector s within cluster: advance s sectors from base_phys.
                # sectors are packed 4/page; when crossing a page boundary,
                # need to skip the 64-byte spare.
                abs_sec_idx = s  # sector index from base
                # base_phys points to one specific (pg, sip) pair
                # Compute pg and sip for that base:
                base_pg_start = (base_phys // PAGE_SIZE) * PAGE_SIZE
                base_sip      = (base_phys - base_pg_start) // BPS
                new_sip = base_sip + s
                new_pg  = (base_phys // PAGE_SIZE) + new_sip // SECS_PER_PAGE
                final_sip = new_sip % SECS_PER_PAGE
                phys_off = new_pg * PAGE_SIZE + final_sip * BPS
                seg = d[phys_off : phys_off + BPS]
            else:
                seg = linear_read_sec(first_data_sec + (c - 2) * spc + s)
            parts.append(seg + b'\xFF' * (BPS - len(seg)) if len(seg) < BPS else seg)
        return b''.join(parts)

    def read_chain_data(start_c, file_size=None):
        buf = b''.join(read_cluster_data(c) for c in cluster_chain(start_c))
        return buf[:file_size] if file_size is not None else buf

    def read_dir_raw(cluster):
        if cluster == 0:
            return linear_read_secs(root_sec, (root_cnt * 32 + BPS - 1) // BPS)
        return read_chain_data(cluster)

    # ------------------------------------------------------------------ #
    # Phase 5: parse directory                                             #
    # ------------------------------------------------------------------ #
    def parse_dir(raw):
        entries, lfn_slots = [], {}
        i = 0
        while i + 32 <= len(raw):
            e = raw[i:i+32]
            if e[0] == 0x00:
                break
            if e[0] == 0xE5:
                lfn_slots = {}
                i += 32
                continue
            attr = e[11]
            if attr == 0x0F:
                seq   = e[0] & 0x3F
                part  = e[1:11] + e[14:26] + e[28:32]
                chars = part.decode('utf-16le', errors='replace')
                chars = chars.rstrip('\uFFFF').rstrip('\x00')
                lfn_slots[seq] = chars
                i += 32
                continue
            long_name = ''.join(lfn_slots[k] for k in sorted(lfn_slots)) if lfn_slots else None
            lfn_slots = {}
            n83 = e[0:8].decode('ascii', 'replace').rstrip()
            x83 = e[8:11].decode('ascii', 'replace').rstrip()
            short = (n83 + ('.' + x83 if x83 else '')).strip()
            name  = long_name if long_name else short
            clus  = (struct.unpack_from('<H', e, 20)[0] << 16) | struct.unpack_from('<H', e, 26)[0]
            size  = struct.unpack_from('<I', e, 28)[0]
            is_dir = bool(attr & 0x10)
            entries.append({'name': name, 'cluster': clus, 'size': size, 'is_dir': is_dir})
            i += 32
        return entries

    # ------------------------------------------------------------------ #
    # Phase 6: walk directory tree                                         #
    # ------------------------------------------------------------------ #
    found_files = {}

    def walk(cluster, path='/', depth=0):
        raw     = read_dir_raw(cluster)
        entries = parse_dir(raw)
        for e in entries:
            n = e['name']
            if not n or n in ('.', '..'):
                continue
            full = path.rstrip('/') + '/' + n
            if e['is_dir']:
                sys.stdout.write('  ' * depth + f'[D] {full}/\n')
                sys.stdout.flush()
                if depth < 8:
                    walk(e['cluster'], full + '/', depth + 1)
            else:
                sys.stdout.write('  ' * depth + f'[F] {full}  ({e["size"]} B, clus={e["cluster"]})\n')
                sys.stdout.flush()
                found_files[full] = e

    print('\n[nand_fat] Directory tree:\n')
    walk(0, '/')

    # ------------------------------------------------------------------ #
    # Phase 7: extract mms45.dat                                           #
    # ------------------------------------------------------------------ #
    matches = {k: v for k, v in found_files.items() if k.lower().endswith('mms45.dat')}
    if matches:
        for path, e in matches.items():
            print(f'\n[nand_fat] Extracting: {path}  size={e["size"]}')
            data = read_chain_data(e['cluster'], e['size'])
            os.makedirs(OUT_DIR, exist_ok=True)
            out_path = os.path.join(OUT_DIR, 'mms45.dat')
            with open(out_path, 'wb') as f:
                f.write(data)
            print(f'[nand_fat] Saved to {out_path}  ({len(data)} bytes)')
    else:
        print('\n[nand_fat] mms45.dat NOT found in any directory.')
        print('[nand_fat] All .dat files:')
        for k in sorted(found_files):
            if k.lower().endswith('.dat'):
                e = found_files[k]
                print(f'  {k}  ({e["size"]} B)')


if __name__ == '__main__':
    main()
