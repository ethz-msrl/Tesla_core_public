function [ mag_system ] = createMagneticSystemFromYAML( filename )
%CREATEMAGNETICSYSTEMFROMYAML creates a MagneticSystem object from a YAML
%definition
% needs the yamlmatlab toolbox in path

definition = ReadYaml(filename);

num_coils = length(definition.Coil_List);

coil_structs = struct('A', {}, 'B', {}, 'P',{}, 'Z', {}, 'numCoeff', {},...
    'CalMeanPercentError', {});

for i=1:num_coils
    coil_name = definition.Coil_List{i};
    coil = definition.(coil_name);
    num_sources = length(coil.Source_List);
    
    % get number of coefficients
    na = length(coil.(coil.Source_List{1}).A_Coeff);
    nb = length(coil.(coil.Source_List{1}).B_Coeff);
    
    A_coeff = zeros(na, num_sources);
    B_coeff = zeros(nb, num_sources);
    
    positions = zeros(3, num_sources);
    directions = zeros(3, num_sources);
    
    for j=1:num_sources
        source_name = coil.Source_List{j};
        source = coil.(source_name);
        A_coeff(:, j) = cell2mat(source.A_Coeff);
        B_coeff(:, j) = cell2mat(source.B_Coeff);
        positions(:, j) = cell2mat(source.Source_Position);
        directions(:, j) = cell2mat(source.Source_Direction);
    end
    
    coil_structs(i) = MagneticSystem.makeCoilStruct(A_coeff, B_coeff, positions, directions);
end

mag_system = MagneticSystem(coil_structs, cell2mat(definition.Workspace_Dimensions), ...
    definition.System_Name);

end

