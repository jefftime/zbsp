const scanner = @import("scanner.zig");
const util = @import("util.zig");
const std = @import("std");
const limits = @import("limits.zig");
const map = @import("map.zig");

comptime {
    const builtin = @import("builtin");
    if (!builtin.is_test) @export(&main, .{ .name = "main" });
}

pub fn main(_: c_int, _: [*c][*c]c_char) callconv(.c) c_int {
    var al = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer al.deinit();

    var file = std.fs.cwd().openFile(
        "mapfile.map",
        .{ .mode = .read_only },
    ) catch |err| util.die(err, "Failed to open mapfile.map", .{});
    const buf = file.readToEndAlloc(
        al.allocator(),
        limits.MAX_FILE_SIZE,
    ) catch |err| util.die(err, "Failed to read mapfile.map", .{});

    const tokens = scanner.scan_buf(
        al.allocator(),
        buf,
        .{ .double_slash_comments = true },
    ) catch |err| util.die(err, "Failed to scan file", .{});
    file.close();

    _ = map.parse(tokens.items) catch |err| util.die(
        err,
        "Failed to parse map",
        .{},
    );

    return 0;
}

test {
    std.testing.refAllDecls(@This());
}
