/* SPDX-License-Identifier: Apache-2.0
 * Last object of every .geapp program (the toolchain ships no crtendS): the
 * zero-length entry that ends .eh_frame, where the unwinder stops reading the
 * program's frame descriptions once Home registers them. */
	.section .eh_frame,"a",@progbits
	.4byte 0
	.section .note.GNU-stack,"",@progbits
