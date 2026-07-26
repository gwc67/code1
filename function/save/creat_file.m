%输出文件
function outPath = creat_file(mat,col_name,csv_path_raw,opts)
    if nargin < 4 || isempty(opts)
        opts = struct('output_dir',[],'output_name',[]);
    end
    if isempty(opts.output_dir)
        outDir = fileparts(csv_path_raw);
    else 
        outDir = char(opts.output_dir);                 %输出的文件夹的位置
    end
    if isempty(outDir)
        outDir = '.';
    end

    if isempty(opts.output_name)
        [~,baseName,~] = fileparts(csv_path_raw);
        outName = [ char(baseName),'_create'];
    else
        outName = char(opts.output_name);
    end

    outPath = char(fullfile(outDir,[outName,'.csv'])); %输出的文件名字

    [folder_out,~,~] = fileparts(outPath);

    %检查文件夹是否存在
    if ~exist(folder_out,'dir')
        mkdir(folder_out);
        fprintf('创建输出文件夹:%s\n',folder_out);
    end

    
    % 构造 table 并写入
    varData = cell(1,size(mat,2));
    for c = 1 : size(mat,2)
        varData{c} = mat(:,c);
    end
    T = table(varData{:}, 'VariableNames', col_name);
    writetable(T,outPath);
end
