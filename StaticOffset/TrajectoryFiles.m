function  TrajectoryFiles(Q1,Q2)
global Path
MRK={'LASI' 'RASI' 'LPSI' 'RPSI',...
    'LTHI' 'LKNE' 'LTIB' 'LANK' 'LHEE' 'LTOE' ,...
    'RTHI' 'RKNE' 'RTIB' 'RANK' 'RHEE' 'RTOE' ,...
    'LTRO' 'LTHI_B' 'LTHI_F' 'LTIB_B' 'LTIB_F' 'LANK_MED' 'LKNE_MED',...
    'RTRO' 'RTHI_B' 'RTHI_F' 'RTIB_B' 'RTIB_F' 'RANK_MED' 'RKNE_MED',...
    'LSHO' 'RSHO' 'CLAV' 'STRN'};

% % Static________________________________________________________________
[num, txt]=xlsread('TRCformatStatic.xlsx');
row4=[]; row5=[];
for i=1:size(txt,2)
    row4=strcat(row4,txt(4,i),'\t');
    row5=strcat(row5,txt(5,i),'\t');
end
row4=strcat(row4, '\n');
row5=strcat(row5, '\n');


excel=[];
for k=1:length(MRK)
    if isfield(Q2.Static(1).Trajectory,(MRK{k}))
    if ~isempty(Q2.Static(1).Trajectory.(MRK{k}))
        excel=[excel Q2.Static(1).Trajectory.(MRK{k})(:,1:3)];
    else
        excel=[excel nan(size(excel,1),3)];
    end
    else
        excel=[excel nan(size(excel,1),3)];
    end
end
filename=strcat(Path,'\Report\Opensim\',Q2.Name,'_',Q2.Static(1).Events.FileName,'.xlsx');
% xlswrite(filename,excel);
N=excel;
dir=1;   % Forwarddirection =1 backwarddirection=-1
for i=1:size(excel,2)/3
    N(:,3*i-2) = dir*excel(:,3*i-2);
    N(:,3*i) = -dir*excel(:,3*i-1);
    N(:,3*i-1) = excel(:,3*i);
end
[r, c]=find(N==0);
for i=1:size(r,1)
    N(r(i),c(i))=nan;
end

xlswrite(filename,txt,'Sheet1');
xlswrite(filename,num(1,2),'Sheet1','B1');
xlswrite(filename,num(3,:),'Sheet1','A3');
xlswrite(filename,txt(3,5),'Sheet1','E3');
xlswrite(filename,num(6:end,1:2),'Sheet1','A6:B54');
xlswrite(filename,N(1:49,:),'Sheet1','C6');

%open the file
newfilename = strrep(filename,'xlsx','trc');
fid_1 = fopen(newfilename,'w');

% first write the header data
fprintf(fid_1,'PathFileType\t4\t(X/Y/Z)\tStatic.trc \n');
fprintf(fid_1,'DataRate\tCameraRate\tNumFrames\tNumMarkers\tUnits\tOrigDataRate\tOrigDataStartFrame\tOrigNumFrames\n');
fprintf(fid_1,'%d \t%d \t%d \t%d \tmm \t%d \t%d \t%d \n', num(3,1), num(3,2), num(3,3), num(3,4), num(3,6), num(3,7) ,num(3,8));
fprintf(fid_1, char(row4));
fprintf(fid_1, char(row5));

% then write the output marker data
for i=1:49
    fprintf(fid_1,'%d\t%f\t', num(5+i,1),num(5+i,2));
    for j=1:size(N,2)
        if isnan(N(i,j))
            fprintf(fid_1,'\t');
        else
            fprintf(fid_1,'%f\t', N(i,j));
        end
    end
    fprintf(fid_1, '\n');
end

% close the file
fclose(fid_1);
clear filename

% Gait__________________________________________________________________
% LR={'Left' 'Right'};
[num, txt]=xlsread('TRCformatGait.xlsx');
row4=[]; row5=[];
for i=1:size(txt,2)
    row4=strcat(row4,txt(4,i),'\t');
    row5=strcat(row5,txt(5,i),'\t');
end
row4=strcat(row4, '\n');
row5=strcat(row5, '\n');

Fr=num(6:end,1:2);
for j=1:size(Q1.Events,2)
    if ~contains(Q1.Events(j).FileName,lower('Static'))
        excel=[];
        for k=1:length(MRK)
            if isfield(Q1.Trajectory(j),(MRK{k}))
            if ~isempty(Q1.Trajectory(j).(MRK{k}))
                excel=[excel Q1.Trajectory(j).(MRK{k})(:,1:3)];
            else
                excel=[excel nan(size(excel,1),3)];
            end
            else
                excel=[excel nan(size(excel,1),3)];
            end
        end
        filename=strcat(Path,'\Report\Opensim\',Q1.Name,'_',Q1.Events(j).FileName,'.xlsx');
        %             xlswrite(filename,excel);
        
        N=excel;
        if excel(1,1)<excel(end,1)
            dir=1;   % Forwarddirection =1 backwarddirection=-1
        else
            dir=-1;
        end
        for i=1:size(excel,2)/3
            N(:,3*i-2) = dir*excel(:,3*i-2);
            N(:,3*i) = -dir*excel(:,3*i-1);
            N(:,3*i-1) = excel(:,3*i);
        end
        if size(Fr,1)>size(N,1)
            Fr2=Fr(1:size(N,1),:);
        else
            Fr2=Fr;
        end
        [r, c]=find(N==0);
        for i=1:size(r,1)
            N(r(i),c(i))=nan;
        end
        
        row3=num(3,:); row3(3)=size(N,1); row3(8)=size(N,1);
        xlswrite(filename,txt,'Sheet1');
        xlswrite(filename,num(1,2),'Sheet1','B1');
        xlswrite(filename,row3,'Sheet1','A3');
        xlswrite(filename,txt(3,5),'Sheet1','E3');
        xlswrite(filename,Fr2,'Sheet1','A6');
        xlswrite(filename,N,'Sheet1','C6');
        
        
        %open the file
        newfilename = strrep(filename,'xlsx','trc');
        fid_1 = fopen(newfilename,'w');
        
        % first write the header data
        fprintf(fid_1,'PathFileType\t4\t(X/Y/Z)\tGait.trc \n');
        fprintf(fid_1,'DataRate\tCameraRate\tNumFrames\tNumMarkers\tUnits\tOrigDataRate\tOrigDataStartFrame\tOrigNumFrames\n');
        fprintf(fid_1,'%d \t%d \t%d \t%d \tmm \t%d \t%d \t%d \n', row3(1), row3(2), row3(3), row3(4), row3(6), row3(7) ,row3(8));
        fprintf(fid_1, char(row4));
        fprintf(fid_1, char(row5));
        
        % then write the output marker data
        for i=1:size(N,1)
            fprintf(fid_1,'%d\t%f\t', Fr2(i,1),Fr2(i,2));
            for k=1:size(N,2)
                if isnan(N(i,k))
                    fprintf(fid_1,'\t');
                else
                    fprintf(fid_1,'%f\t', N(i,k));
                end
            end
            fprintf(fid_1, '\n');
        end
        
        % close the file
        fclose(fid_1);
        
    end
end
end
% %% Force Analysis
% % Gait__________________________________________________________________
% LR={'Left' 'Right'};
% [num, txt]=xlsread('TRCformatGait.xlsx');
% Fr=num(6:end,1:2);
% for h=1:length(LR)
%     for j=1:size(Q2.ForcePlate.(LR{h}),2)
%         if ~isempty(Q2.ForcePlate.(LR{h})(j).Force)
%             excel=[];
%             for k=1:length(MRK)
%                 if ~isempty(Q2.Trajectory.(LR{h})(j).(MRK{k}))
%                 excel=[excel Q2.Trajectory.(LR{h})(j).(MRK{k})(:,1:3)];
%                 else
%                     excel=[excel zeros(size(excel,1),3)];
%                 end
%             end
%             filename=strcat(Q2.Name,'_',LR{h},'_',Q2.Events.(LR{h})(j).TrialName,'.xlsx');
%             %             xlswrite(filename,excel);
%
%             N=excel;
%             if excel(1,1)<excel(end,1)
%                 dir=1;   % Forwarddirection =1 backwarddirection=-1
%             else
%                 dir=-1;
%             end
%             for i=1:size(excel,2)/3
%                 N(:,3*i-2) = dir*excel(:,3*i-2);
%                 N(:,3*i) = -dir*excel(:,3*i-1);
%                 N(:,3*i-1) = excel(:,3*i);
%             end
%             if size(Fr,1)>size(N,1)
%                 Fr2=Fr(1:size(N,1),:);
%             else
%                 Fr2=Fr;
%             end
%             row3=num(3,:); row3(3)=size(N,1); row3(8)=size(N,1);
%             xlswrite(filename,txt,'Sheet1');
%             xlswrite(filename,num(1,2),'Sheet1','B1');
%             xlswrite(filename,row3,'Sheet1','A3');
%             xlswrite(filename,txt(3,5),'Sheet1','E3');
%             xlswrite(filename,Fr2,'Sheet1','A6');
%             xlswrite(filename,N,'Sheet1','C6');
%         end
%     end
% end
%
%
% %% ForcePlate Files
% j=[1 1];  h=1;
% j1=j(h);
% j(h)=[]; j2=j;
% FN=fieldnames(Q2.ForcePlate.Right);
% if h==1
%     EVcon={'RFC1' 'LFC' 'RFC2' 'RTO'};
%     EVips={'LFC1' 'RFC' 'LFC2' 'RTO'};
%     h1=1;h2=2;
%     HEE={'RHEE'};
% else
%     EVcon={'LFC1' 'RFC' 'LFC2' 'LTO'};
%     EVips={'RFC1' 'LFC' 'RFC2' 'LTO'};
%     h1=2; h2=1;
%     HEE={'LHEE'};
% end
% FPips=[]; FPcon=[];
% for k=1:length(FN)
%     FPips=[FPips Q2.ForcePlate.(LR{h1})(j1).(FN{k})];
%     FPcon=[FPcon Q2.ForcePlate.(LR{h2})(j2).(FN{k})];
% end
% Tcon2=(Q2.Events.(LR{h2})(j2).(EVcon{3})-Q2.Events.(LR{h2})(j2).(EVcon{2}))*10;
% Tcon1=(Q2.Events.(LR{h2})(j2).(EVcon{2})-Q2.Events.(LR{h2})(j2).(EVcon{1})+1)*10;
%
% Tips2=(Q2.Events.(LR{h1})(j1).(EVips{3})-Q2.Events.(LR{h1})(j1).(EVips{2}))*10;
% Tips1=(Q2.Events.(LR{h1})(j1).(EVips{2})-Q2.Events.(LR{h1})(j1).(EVips{1})+1)*10;
%
% FPcon1=FPcon(1:Tcon1,:);
% FPcon2=FPcon(Tcon1+1:end,:);
%
% if Q2.Events.(LR{h1})(j1).(EVips{1})<Q2.Events.(LR{h2})(j2).(EVcon{1})
%     t=(Q2.Events.(LR{h2})(j2).(EVcon{4})-Q2.Events.(LR{h2})(j2).(EVcon{2}))*10;
%     COP=FPcon2(1:t,7:8);
%     t1=Q2.Events.(LR{h2})(j2).(EVcon{2})-Q2.Events.(LR{h2})(j2).(EVcon{1});
%     t2=Q2.Events.(LR{h2})(j2).(EVcon{4})-Q2.Events.(LR{h2})(j2).(EVcon{1});
%     heel=Q2.Trajectory.(LR{h2})(j2).(HEE{1})(t1:t2,1:2);
%     heel=MinaSize(heel,size(COP,1));
%     vec=COP-heel;
%     t3=Q2.Events.(LR{h1})(j1).(EVips{4})-Q2.Events.(LR{h1})(j1).(EVips{1});
%     heel2=Q2.Trajectory.(LR{h1})(j1).(HEE{1})(1:t3,1:2);
%     heel2=MinaSize(heel2,size(COP,1));
%     COP2=heel2+vec;
%     FPcon2(1:t,7:8)=COP2;
%
%     excel=[FPips [MinaSize(FPcon2,Tips2); MinaSize(FPcon1,Tips1)]];
%     if Q2.Trajectory.(LR{h1})(j1).LASI(1,1)>Q2.Trajectory.(LR{h1})(j1).LASI(end,1)
%         excel2=[excel(:,10:end) excel(:,1:9)];
%         excel=excel2;
%     end
% else
%     t=(Q2.Events.(LR{h2})(j2).(EVcon{2})-Q2.Events.(LR{h2})(j2).(EVcon{1}))*10;
%     COP=FPcon1(1:t,7:8);
%     t1=t/10;
%     heel=Q2.Trajectory.(LR{h2})(j2).(HEE{1})(1:t1,1:2);
%     heel=MinaSize(heel,size(COP,1));
%     vec=COP-heel;
%     t2=Q2.Events.(LR{h1})(j1).(EVips{2})-Q2.Events.(LR{h1})(j1).(EVips{1});
%     t3=Q2.Events.(LR{h1})(j1).(EVips{3})-Q2.Events.(LR{h1})(j1).(EVips{1});
%     heel2=Q2.Trajectory.(LR{h1})(j1).(HEE{1})(t2:t3,1:2);
%     heel2=MinaSize(heel2,size(COP,1));
%     COP2=heel2+vec;
%     FPcon1(1:t,7:8)=COP2;
%
%     excel=[FPips [MinaSize(FPcon2,Tips2); MinaSize(FPcon1,Tips1)]];
%     if Q2.Trajectory.(LR{h1})(j1).LASI(1,1)<Q2.Trajectory.(LR{h1})(j1).LASI(end,1)
%         excel2=[excel(:,10:end) excel(:,1:9)];
%         excel=excel2;
%     end
% end
% filename=strcat(Q2.Name,'_',LR{h},'_',Q2.Events.(LR{h})(j).TrialName,'_GRF.xlsx');
%
% N=excel;
% if Q2.Trajectory.(LR{h1})(j1).LASI(1,1)<Q2.Trajectory.(LR{h1})(j1).LASI(end,1)
%     dir=1;
% else
%     dir=-1;
% end
%
% N(:,1)=-dir*excel(:,1);
% N(:,2)=-excel(:,3);
% N(:,3)=dir*excel(:,2);
% N(:,4)=dir*excel(:,7)/1000;
% N(:,5)=excel(:,9)/1000;
% N(:,6)=-dir*excel(:,8)/1000;
% N(:,7)=-dir*excel(:,10);
% N(:,8)=-excel(:,12);
% N(:,9)=dir*excel(:,11);
% N(:,10)=dir*excel(:,16)/1000;
% N(:,11)=excel(:,18)/1000;
% N(:,12)=-dir*excel(:,17)/1000;
% for i=1:size(excel,1)
%     N(i,13)=0;
%     if excel(i,3)==0
%         N(i,14)=0;
%     else
%         N(i,14)=-(excel(i,6)+excel(i,5)/excel(i,3)*excel(i,2)+excel(i,4)/excel(i,3)*excel(i,1))/1000;
%     end
%     N(i,15)=0;
%     N(i,16)=0;
%     if excel(i,12)==0
%         N(i,17)=0;
%     else
%         N(i,17)=-(excel(i,15)+excel(i,14)/excel(i,12)*excel(i,11)+excel(i,13)/excel(i,12)*excel(i,10))/1000;
%     end
%     N(i,18)=0;
% end
%
% [num, txt]=xlsread('MOTformatGRF.xlsx');
% if size(num,1)>size(N,1)
%     num2=num(1:size(N,1));
%     num=num2;
% end
% row3={strcat('nRows=',num2str(size(num,1)))};
% xlswrite(filename,txt,'Sheet1');
% xlswrite(filename,row3,'Sheet1','A3');
% xlswrite(filename,num,'Sheet1','A8');
% xlswrite(filename,N,'Sheet1','B8');