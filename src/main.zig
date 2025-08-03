const scanner = @import("scanner.zig");
const util = @import("util.zig");
const std = @import("std");
const limits = @import("limits.zig");
const map = @import("map.zig");
const bsp = @import("bsp.zig");

comptime {
    const builtin = @import("builtin");
    if (!builtin.is_test) @export(&main, .{ .name = "main" });
}

pub fn main(_: c_int, _: [*c][*c]c_char) callconv(.c) c_int {
    var arena_allocator = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena_allocator.deinit();
    const al = arena_allocator.allocator();

    const filepath = "data/pyramid.map";
    // const filepath = "data/unnamed.map";
    // const filepath = "data/mapfile.map";
    // const filepath = "data/3cube.map";
    // const filepath = "data/cubewall.map";
    var file = std.fs.cwd().openFile(
        filepath,
        .{ .mode = .read_only },
    ) catch |err| util.die(err, "Failed to open {s}", .{filepath});
    const buf = file.readToEndAlloc(
        al,
        limits.MAX_FILE_SIZE,
    ) catch |err| util.die(err, "Failed to read mapfile.map", .{});

    const tokens = scanner.scan_buf(
        al,
        buf,
        .{ .double_slash_comments = true },
    ) catch |err| util.die(err, "Failed to scan file", .{});
    file.close();

    const m = map.parse(al, tokens.items) catch |err| util.die(
        err,
        "{}: Failed to parse map",
        .{err},
    );
    // m.print();

    const b = bsp.compile(al, m) catch |err| util.die(
        err,
        "Failed to compile BSP",
        .{},
    );
    _ = b;
    // b.print_digraph();

    // _ = arena_allocator.reset(.free_all);

    return 0;
}

test {
    std.testing.refAllDecls(@This());
}
