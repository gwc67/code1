function result = my_flitter(csvPath,options)

    
    if nargin < 1 || isempty(csvPath)
        csvPath = fullfile(fileparts(mfilename("fullpath")),'..','excel_csv','still_circle.csv');           %为什么制定的是still_circle.csv?
        disp("启用默认配置文件")
        csvPath = char(csvPath);
    end     
    csvPath = char(csvPath);
    if nargin < 2 || isempty(options)
        options = struct();            % 如果调用时不传第二个参数，或者传了空，就手动创建一个空结构体；
    end
              
    opts.output_dir = '';
    opts.output_name = '';

    flds = fieldnames(options);         
    for i = 1 : length(flds)            %循环覆盖默认参数
        oldvalue = opts.(flds{i});
        opts.(flds{i}) = options.(flds{i});
        fprintf('更新参数[%s] : 旧值= %g -> 新值=%g\n',flds{i},oldvalue,opts.(flds{i}));
    end


    %%
    [rawData,col_raw_names] = loadRawCsv(csvPath);

    row_raw_num = size(rawData,1); 
    col_raw_num = size(rawData,2);
    fprintf('   原始数据 ： %d 行 x %d 列  \n',row_raw_num,col_raw_num);

    tickCol = rawData(:, 20);  % U20 = Tick
    
    [data_de, row_de_num] = deduplicateByTick(rawData, tickCol);

    fprintf('   去重后: %d 行 (删除 %d 行重复)\n', row_de_num, row_raw_num - row_de_num);


    %%将时间列变得更加平滑
    tick_abs = data_de(:,20);  
    t_start = tick_abs(1);
    tick_rel   = tick_abs - t_start; 
    
    fprintf(  'Tick 范围： %.3f - %.3f s\n',tick_abs(1),tick_abs(end));
    fprintf(   '相对范围:  %.3f - %.3f s\n',tick_rel(1),tick_rel(end));




    %% 构造输出14列得矩阵
    radar_pos = data_de(:,1:3); 
    target_pos = data_de(:,4:6);
    cmd_vel    = data_de(:,7:9);
    
    qua = data_de(:,10:13);
    rt_tar = data_de(:,17:18);
    error = target_pos - radar_pos;

    % t_rel_out = data_de(:,21);   %均匀时间列
    % t_abs_out = data_de(:,22);   %绝对时间列

    mat_out = [tick_rel,tick_abs,radar_pos,target_pos,cmd_vel,error,qua,rt_tar];
    
    col_names = {'T_REL','T_ABS',...
                'RADAR_POS_X','RADAR_POS_Y','RADAR_POS_Z',...
                'TARGET_POS_X','TARGET_POS_Y','TARGET_POS_Z',...
                'CMD_SPEED_X','CMD_SPEED_Y','CMD_SPEED_Z',...
                'ERROR_X','ERROR_Y','ERROR_Z','qw','qx','qy','qz','rt_tar_vel_x','rt_tar_vel_y'};
    outPath = saveOutput(mat_out,col_names,csvPath,opts);
    fprintf('  写入:%s \n',outPath);

    
    result.output_path   = outPath;
    result.data          = mat_out;
    result.row_raw_num   = row_raw_num;
    result.row_de_num    = row_de_num;
    result.output_re_num = row_de_num;


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
    opts.DataLines = [3,Inf];                   %数据开始列，在第3行
    opts = setvartype(opts,1:nCols,'double');
    data = readmatrix(csvPath,opts);

    %最后的Tick
    lastCol = data(:,end);
    if all(isnan(lastCol)) || (max(lastCol) < 1 && min(lastCol) > -1)
        warning('最后一列异常')
    end

    % fprintf(colNames);
    disp('CVS 列名')
    disp(colNames)
    %
    data(all(isnan(data),2),:) = [];

end


function [data_de,data_de_num] = deduplicateByTick(data_raw,tickCol)
    N = size(data_raw , 1);     %返回行数 size（data，2）返回列数
    if N == 0
        data_de = data_raw;
        data_de_num = 0;
        return;
    end

    %提取tickCol 数组里面 不重复的元素
    % 例：原 tickCol = [10,10,20,20,20,30]
    % uniqueTicks = [10,20,30]
    % tickCol = [10,10,20,20,20,30]
    % groupIdx = [1,1,2,2,2,3]
    % 和输入tickCol长度完全一样，每一行数字代表「这一行属于第几个唯一 Tick 分组」
    [uniqueTciks, ~ , groupIdx_raw] = unique(tickCol,'stable');

    %{
    zeros(M,1)：创建 M 行 1 列 的全 0 列向量。 
    %}

    uniqueTicks_num = length(uniqueTciks);
    keepIdx = zeros(uniqueTicks_num,1);                 %keepidx保留下来的csv波形
    for k = 1:uniqueTicks_num
        all_cur_group_row = find(groupIdx_raw == k);    %% 输出 all_cur_group_row = [1,2]

        cmd_speed = data_raw(all_cur_group_row,7:9);
        has_speed = any(abs(cmd_speed) > 0.5,2);        %对每一行，只要 3 个速度里任意一列满足 > 0.5，该行结果为 true

        if any(has_speed)
            speedRows = all_cur_group_row(has_speed);
            keepIdx(k) = speedRows(1); 
        else 
            keepIdx(k) = all_cur_group_row(end);
        end
    end

    keepIdx = sort(keepIdx);
    data_de = data_raw(keepIdx,:);                        %数据矩阵
    data_de_num   = length(keepIdx);                         %矩阵有效长度
end

%输出文件
function outPath = saveOutput(mat,col_name,csv_path_raw,opts)
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
        outName = [ char(baseName),'_filtered'];
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


