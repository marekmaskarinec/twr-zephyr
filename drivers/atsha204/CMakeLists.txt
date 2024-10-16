
zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ATSHA204 atsha204.c)
