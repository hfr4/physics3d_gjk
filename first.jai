#import "Basic";
#import "Compiler";
#import "Process";

#run {
    set_build_options_dc(.{do_output = false});

    w := compiler_create_workspace();

    options := get_build_options(w);

    options.output_executable_name              = "a";
    options.output_type                         = .EXECUTABLE;  
    options.dead_code_elimination               = .NONE;
    options.text_output_flags                   = 0;
    options.backend                             = .X64;
    options.import_path                         = .["C:/jai/modules", "C:/jai/my_modules"];
    options.emit_debug_info                     = .DEFAULT;
    options.additional_linker_arguments         = .[ "/NODEFAULTLIB", "/NOIMPLIB", "/NOEXP", "/IGNORE:4286" ];
    options.shorten_filenames_in_error_messages = true;

    set_build_options(options, w);

    success := false;

    compiler_begin_intercept(w);

    add_build_file("main.jai", w);

    while true {                     
        message := compiler_wait_for_message();  
        if !message break;

        if message.kind == .COMPLETE {   
            success = (cast(*Message_Complete) message).error_code == 0;
            break;
        }
    }

    compiler_end_intercept(w);   

    if success then run_command(options.output_executable_name);
}
