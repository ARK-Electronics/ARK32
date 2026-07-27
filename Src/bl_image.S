/*
 * Embedded bootloader image for the app-side bootloader update.
 *
 * The bootloader is committed as a .bin (see Bootloaders/README.md) and pulled
 * in verbatim with .incbin, so the bytes the linker places are provably the
 * same artifact the AM32-bootloader build produced - a hex-dumped C array is
 * not verifiable against a release, a .bin is.
 *
 * Assembling the blob here rather than compiling a generated C array also keeps
 * it out of the LTO IR entirely: it reaches the linker as an ordinary object,
 * so -flto cannot discard or relocate it and no used/KEEP attribute is needed
 * on the C side to hold it down.
 *
 * BL_IMAGE_FILE and BL_REGION_SIZE come from the Makefile. The bounds symbols
 * are defined by the linker script rather than here, so they exist in builds
 * that do not embed an image too (start == end, so the length is zero and
 * maybe_update_bootloader() returns early).
 */

#if defined(EMBED_BOOTLOADER)

	.section .bl_image, "a", %progbits
	.balign 4

__bl_image_data:
	.incbin BL_IMAGE_FILE

	.if (. - __bl_image_data) > BL_REGION_SIZE
	.error "bootloader image is larger than the bootloader region"
	.endif

	/*
	 * Pad out to the full region with the erased-flash value. The compare in
	 * maybe_update_bootloader() then covers every byte it erases, so the region
	 * converges to exactly this image instead of leaving a tail of whatever the
	 * previous bootloader wrote there.
	 */
	.space BL_REGION_SIZE - (. - __bl_image_data), 0xFF

#endif /* EMBED_BOOTLOADER */
