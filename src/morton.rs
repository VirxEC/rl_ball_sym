fn expand3(a: u64) -> u64 {
    let mut b = a as u128;

    b = (b * 0x0000000100000001) & 0x001F00000000FFFF;
    b = (b * 0x0000000000010001) & 0x001F0000FF0000FF;
    b = (b * 0x0000000000000101) & 0x100F00F00F00F00F;
    b = (b * 0x0000000000000011) & 0x10C30C30C30C30C3;
    b = (b * 0x0000000000000005) & 0x1249249249249249;

    b as u64
}

pub fn encode(x: u64, y: u64, z: u64) -> u64 {
    (expand3(z) << 2) + (expand3(y) << 1) + expand3(x)
}