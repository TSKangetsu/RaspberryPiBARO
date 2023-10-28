aux_source_directory(. BAROMAIN)
aux_source_directory(MS5611 BAROSOURCE)
add_library(LIBBARO ${BAROSOURCE} ${BAROMAIN})