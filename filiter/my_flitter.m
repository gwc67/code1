function result = pre_log(csvPath)

    
    if nargin < 1 || isempty(csvPath)
        csvPath = fullfile(fileparts(mfilename("fullpath")),'..','excel_csv','still_circle.csv');           %为什么制定的是still_circle.csv?
        csvPath = char(csvPath);
    end     


    [rawData,colNames] = loadRawCsv(csvPath);

end

function [data,colNames] = loadRawCsv(csvPath)

    fid = fopen(csvPath,'r');           %指只读，而不是红色
    if fid == -1
        error('无法打开文件：%s',csvPath);
    end

     % 跳过第 1 行（数字 header） CSV波形上的第一行
    fgetl(fid);

    %第二行是列名
    headerline = fgetl(fid);
    fclose(fid);    

    %解析列名？？？ 这是怎么解析的？
    colNames = strsplit(headerline,',');        %根据头文件的‘，’进行分离
    colNames = strtrim(colNames);
    colNames(cellfun(@isempty,colNames)) = [];  %去除末尾空列

    nCols = length(colNames);
    opts = detectImportOptions(csvPath,'NumVariables',nCols);
    opts.DataLines = [3,Inf];
    opts = setvartype(opts,1:nCols,'double');
    data = readmatrix(csvPath,opts);

    %最后的Tick
    lastCol = data(:,end);
    if all(isnan(lastCol)) || (max(lastCol) < 1 && min(lastCol) > -1)
        warning('最后一列异常')
    end

    fprintf(colNames);
    %
    data(all(isnan(data),2),:) = [];

end