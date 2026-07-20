function result = pre_log(csvPath,options)

    
    if nargin < 1 || isempty(csvPath)
        csvPath = fullfile(fileparts(mfilename("fullpath")),'..','excel_csv','still_circle.csv');           %为什么制定的是still_circle.csv?
        csvPath = char(csvPath);
    end
    csvPath = char(csvPath);
     if nargin < 2 || isempty(options)
        options = struct();
    end        
    opts.dt = 0.1;
    opts.output_dir = '';
    opts.output_name = '';
    flds = fieldnames(options);
    for i = 1 : length(flds)
        opts.(flds{i}) = options.(flds{i});
    end

    [rawData,colNames] = loadRawCsv(csvPath);

end

function [rawData,colNames] = loadRawCsv(csvPath)

    fid = fopen(csvPath,'r');           %指只读，而不是红色
    if fid == -1
        error('无法打开文件：%s',csvPath);
    end

    fgetl(fid);

    headerline = fgetl(fid);
    fclose(fid);

    colNames = strsplit(headerline,',');
    colNames = strtrim(colNames);
    colNames(cellfun(@isempty,colNames)) = []; %去除末尾空列

    nCols = length(colNames);
    opts = detectImportOptions(csvPath,'NumVariables',nCols);
    opts.DataLines = [3,Inf];
    opts = setvartype(opts,1:nCols,'double');
    data = readmatrix(csvPath,opts);

end