function(__fetch_googletest download_module_path download_root)
  set(GOOGLETEST_DOWNLOAD_ROOT ${download_root})

  configure_file(
    ${download_module_path}/googletest-download.cmake
    ${download_root}/CMakeLists.txt
    @ONLY
  )

  unset(GOOGLETEST_DOWNLOAD_ROOT)

  execute_process(
    COMMAND
      "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
    WORKING_DIRECTORY
      ${download_root}
  )

  execute_process(
    COMMAND
      "${CMAKE_COMMAND}" --build .
    WORKING_DIRECTORY
      ${download_root}
  )

  add_subdirectory(
    ${download_root}/googletest-src
    ${download_root}/googletest-build
  )
endfunction()
