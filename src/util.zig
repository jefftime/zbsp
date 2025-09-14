const std = @import("std");

pub fn streq(str1: []const u8, str2: []const u8) bool {
    return std.mem.eql(u8, str1, str2);
}

fn quit(err: ?anyerror) noreturn {
    std.process.exit(if (err) |e| @intCast(@intFromError(e)) else 1);
}

pub fn die(err: ?anyerror, comptime fmt: []const u8, args: anytype) noreturn {
    var bw = std.io.bufferedWriter(std.io.getStdErr().writer());
    const stderr = bw.writer();

    stderr.print("({?}) ", .{err}) catch quit(err);
    stderr.print(fmt, args) catch quit(err);
    stderr.print("\n", .{}) catch quit(err);
    bw.flush() catch quit(err);

    quit(err);
}
