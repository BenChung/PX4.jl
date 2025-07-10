cd(@__DIR__)
import IDLParser
import IDLParser.Parse: open_idl
import IDLParser.ConstResolution: resolve_constants
import IDLParser.Generation: generate_code

interfaces = IDLParser.ConstResolution.ModuleDecl.Type[]
for idl in readdir("../idl"; join=true) 
    if !endswith(idl, r".*\.idl") continue end
    push!(interfaces, resolve_constants(convert(Vector{IDLParser.Parse.Decl}, open_idl(idl)))[1])
end
code = generate_code(interfaces)

output_file = "px4.jl"
open(output_file, "w") do f
    for c in code 
        println(f, c)
    end
end