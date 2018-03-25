file(REMOVE_RECURSE
  "libclara.pdb"
  "libclara.a"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/clara.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
