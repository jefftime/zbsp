const scanner = @import("scanner.zig");

pub const Map = struct {};

pub fn parse(tokens: []scanner.Token) !Map {
    _ = tokens;
    return error.NotImplemented;
}
