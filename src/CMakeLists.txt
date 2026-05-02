# --- Target: Sample Library ---
# Create a static library from the source files
#add_library(sample_lib STATIC )
#target_include_directories(sample_lib PUBLIC .) 

# --- Target: Main Application ---
# Define the main executable produced by the project
#add_executable(sample_app )

# Link internal library to the main application
#target_link_libraries(sample_app sample_lib)

# --- External Dependencies ---
# Locate the Boost library on the host system
#find_package(Boost REQUIRED)
# Link the Boost headers/libraries to main application
#target_link_libraries(sample_app boost::boost)