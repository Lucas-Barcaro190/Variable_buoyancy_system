"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.encodeHexByte = encodeHexByte;
exports.encodeHexBuf = encodeHexBuf;
exports.encodeHexUint32BE = encodeHexUint32BE;
exports.encodeHexUint32 = encodeHexUint32;
exports.decodeHexBuf = decodeHexBuf;
exports.decodeHexUint32Array = decodeHexUint32Array;
exports.decodeHexUint32 = decodeHexUint32;
exports.gdbChecksum = gdbChecksum;
exports.gdbMessage = gdbMessage;
function encodeHexByte(value) {
    return (value >> 4).toString(16) + (value & 0xf).toString(16);
}
function encodeHexBuf(buf) {
    return Array.from(buf).map(encodeHexByte).join('');
}
function encodeHexUint32BE(value) {
    return encodeHexBuf(new Uint8Array([(value >> 24) & 0xff, (value >> 16) & 0xff, (value >> 8) & 0xff, value & 0xff]));
}
function encodeHexUint32(value) {
    const buf = new Uint32Array([value]);
    return encodeHexBuf(new Uint8Array(buf.buffer));
}
function decodeHexBuf(encoded) {
    const result = new Uint8Array(encoded.length / 2);
    for (let i = 0; i < result.length; i++) {
        result[i] = parseInt(encoded.substr(i * 2, 2), 16);
    }
    return result;
}
function decodeHexUint32Array(encoded) {
    return new Uint32Array(decodeHexBuf(encoded).buffer);
}
function decodeHexUint32(encoded) {
    return decodeHexUint32Array(encoded)[0];
}
function gdbChecksum(text) {
    const value = text
        .split('')
        .map((c) => c.charCodeAt(0))
        .reduce((a, b) => a + b, 0) & 0xff;
    return encodeHexByte(value);
}
function gdbMessage(value) {
    return `$${value}#${gdbChecksum(value)}`;
}
