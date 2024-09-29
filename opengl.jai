Vertex :: struct {
    position: Vector3;
    texcoord: Vector2;
    normal:   Vector3;
}

make_vertices :: (path: string) -> [..] Vertex {
    #import "String";

    string_count :: (str: string, substring: string) -> int {
        count       := 0;
        start_index := 0;

        while true {
            index := find_index_from_left(str, substring, start_index);
            if index == -1 break;
            start_index = index + substring.count;
            count += 1;
        }

        return count;
    }

    vertices: [..] Vertex;

    obj_string, ok := read_entire_file(path,, temp); 
    assert(ok);

    positions:  [..] Vector3;
    texcoords:  [..] Vector2;
    normals:    [..] Vector3;
    faces:      [..][3][3] int;

    positions.allocator = temp;
    texcoords.allocator = temp;
    normals.allocator   = temp;
    faces.allocator     = temp;

    lines := split(obj_string, "\n");

    for line: lines {
        words := split(line, " ");

        if words.count == 1 continue;

        if words[0] == {
            case "v";
                array_add(*positions, .{ string_to_float(words[1]), string_to_float(words[2]), string_to_float(words[3]) });
    
            case "vt";
                array_add(*texcoords, .{ string_to_float(words[1]), string_to_float(words[2]) });
    
            case "vn";
                array_add(*normals  , .{ string_to_float(words[1]), string_to_float(words[2]), string_to_float(words[3]) });
    
            case "f";
                // fan triangulation
                for 3..words.count - 1 {
                    face: [3][3] int;

                    triangulated_face_idxs := int.[1, it - 1, it];

                    for idx, j: triangulated_face_idxs {
                        dbl_slash_count := string_count(words[idx], "//");
                        slash_count     := string_count(words[idx], "/");

                        if dbl_slash_count == 1
                        {
                            idxs := split(words[idx], "//");
                            face[j][0] = string_to_int(idxs[0]) - 1;
                            face[j][1] = -1;
                            face[j][2] = string_to_int(idxs[1]) - 1;
                        } 
                        else if slash_count == 2
                        {
                            idxs := split(words[idx], "/");
                            face[j][0] = string_to_int(idxs[0]) - 1;
                            face[j][1] = string_to_int(idxs[1]) - 1;
                            face[j][2] = string_to_int(idxs[2]) - 1;
                        }
                        else if slash_count == 1
                        {
                            idxs := split(words[idx], "/");
                            face[j][0] = string_to_int(idxs[0]) - 1;
                            face[j][1] = string_to_int(idxs[1]) - 1;
                            face[j][2] = -1;
                        }
                        else if slash_count == 0
                        {
                            face[j][0] = string_to_int(words[idx]) - 1;
                            face[j][1] = -1;
                            face[j][2] = -1;
                        }
                    }

                    array_add(*faces, face);
                }
        }
    }

    for face: faces {
        v0, v1, v2: Vertex;

        if face[0][0] != -1  v0.position = positions[face[0][0]];
        if face[1][0] != -1  v1.position = positions[face[1][0]];
        if face[2][0] != -1  v2.position = positions[face[2][0]];

        if face[0][1] != -1  v0.texcoord = texcoords[face[0][1]];
        if face[1][1] != -1  v1.texcoord = texcoords[face[1][1]];
        if face[2][1] != -1  v2.texcoord = texcoords[face[2][1]];

        CALCULATE_VERTEX_NORMALS :: true;
        #if CALCULATE_VERTEX_NORMALS {
            triangle_normal :: (a: Vector3, b: Vector3, c: Vector3) -> Vector3 {
                return normalize(cross(b - a, c - b));
            }

            normal := triangle_normal(v0.position, v1.position, v2.position);
            v0.normal = normal;
            v1.normal = normal;
            v2.normal = normal;
        } else {
            if face[0][2] != -1  v0.normal = normals[face[0][2]];
            if face[1][2] != -1  v1.normal = normals[face[1][2]];
            if face[2][2] != -1  v2.normal = normals[face[2][2]];
        }

        array_add(*vertices, v0, v1, v2);
    }

    return vertices;
}

make_vao :: (vertices: [] Vertex) -> GLuint {
    vao: GLuint;
    vbo: GLuint; 

    glGenVertexArrays(1, *vao);
    glGenBuffers(1, *vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    defer glBindVertexArray(0); 
    defer glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBufferData(GL_ARRAY_BUFFER, size_of(Vertex) * vertices.count, vertices.data, GL_STREAM_DRAW);

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);

    offset: *Vertex;
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, xx size_of(Vertex), xx *offset.position);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, xx size_of(Vertex), xx *offset.texcoord);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, xx size_of(Vertex), xx *offset.normal);

    return vao;
}

create_texture :: (filename: string) -> GLuint {
    stbi :: #import "stb_image";

    texture_w, texture_h, channels: s32;
    texture_data := stbi.stbi_load(temp_c_string(filename), *texture_w, *texture_h, *channels, 0);
    defer stbi.stbi_image_free(texture_data);

    texture: GLuint;
    glGenTextures(1, *texture);

    glBindTexture(GL_TEXTURE_2D, texture);
    defer glBindTexture(GL_TEXTURE_2D, 0);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,     GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,     GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, xx texture_w, xx texture_h, 0, GL_RGBA, GL_UNSIGNED_BYTE, texture_data);

    return texture;
}

make_shader_program :: (filename: string) -> GLuint {
    LOG_BUFFER_SIZE :: 512;

    shader_text := read_entire_file(filename);
    defer free(shader_text.data);

    make_shader_object :: (shader: string, prefix: string, shader_type : GLenum) -> GLuint {
        shader_object := glCreateShader(shader_type);

        shaders: [2] *u8;
        lengths: [2] s32;

        shaders[0] = prefix.data;
        shaders[1] = shader.data;
        lengths[0] = xx prefix.count;
        lengths[1] = xx shader.count;

        glShaderSource(shader_object, 2, shaders.data, lengths.data);

        glCompileShader(shader_object);

        success : GLint;
        glGetShaderiv(shader_object, GL_COMPILE_STATUS, *success);

        if !success then {
            log_data: [LOG_BUFFER_SIZE]u8;
            glGetShaderInfoLog(shader_object, log_data.count, null, log_data.data);
            log("%", to_string(log_data.data), flags=.ERROR);
            return 0;
        }

        return shader_object;
    }

    PREFIX_V :: #string END
    #version 330 core
    #define VERTEX_SHADER
    END

    PREFIX_F :: #string END
    #version 330 core
    #define FRAGMENT_SHADER
    END
    
    shader_v := make_shader_object(shader_text, PREFIX_V, GL_VERTEX_SHADER);
    shader_f := make_shader_object(shader_text, PREFIX_F, GL_FRAGMENT_SHADER);
    defer glDeleteShader(shader_v);
    defer glDeleteShader(shader_f);

    shader_p := glCreateProgram();

    glAttachShader(shader_p, shader_v);
    glAttachShader(shader_p, shader_f);
    glLinkProgram(shader_p);

    success: GLint;
    glGetProgramiv(shader_p, GL_LINK_STATUS, *success);

    if !success then {
        log_data: [LOG_BUFFER_SIZE]u8;
        glGetProgramInfoLog(shader_p, log_data.count, null, log_data.data);
        log("%", to_string(log_data.data), flags=.ERROR);
        return 0;
    } 

    return shader_p;
}
