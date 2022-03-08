function [LocalCo, Mat] = CoordinateTest1(Mat)
%%
global MarkerDiameter
if isempty(MarkerDiameter)
Pr={ 'Marker Diameter (mm):'};
    Res = inputdlg(Pr,'Test Setting',[1 40],{ '9.5'});
    MarkerDiameter=str2num(Res{1});
end

LR={'Left' 'Right'};
LocalCo.Trajectory=Mat.Trajectory;
LEssMRK={'LASI' 'RASI' 'LPSI' 'RPSI' 'LKNE' 'LKNE_MED' 'LANK' 'LANK_MED' 'RANK_MED' 'LTOE'};
REssMRK={'LASI' 'RASI' 'LPSI' 'RPSI' 'RKNE' 'RKNE_MED' 'RANK' 'RANK_MED' 'LANK_MED' 'RTOE'};

for h=1:length(LR)
    clear flag
    if h==1
        EssMRK=LEssMRK;
    else
        EssMRK=REssMRK;
    end
    for t=1:size(Mat.Trajectory.(LR{h}),2)
        Frames=size(Mat.Trajectory.(LR{h})(t).LASI,1);
        flag(t)=0;
        for o=1:length(EssMRK)
            if isfield(Mat.Trajectory.(LR{h})(t),(EssMRK{o}))
            if isempty(Mat.Trajectory.(LR{h})(t).(EssMRK{o}))
                flag(t)=1;
            end
            else
                flag(t)=1;
            end
        end
        if flag(t)==0
            for i=1:Frames %Number Of Frames
                %             a=(Mat.Trajectory.(LR{h})(t).LTOE(i,2)-(Mat.Trajectory.(LR{h})(t).LANK(i,2)+Mat.Trajectory.(LR{h})(t).LANK_MED(i,2))/2)/...
                %                 (Mat.Trajectory.(LR{h})(t).LTOE(i,1)-(Mat.Trajectory.(LR{h})(t).LANK(i,1)+Mat.Trajectory.(LR{h})(t).LANK_MED(i,1))/2);
                %             b=Mat.Trajectory.(LR{h})(t).LTOE(i,2)-Mat.Trajectory.(LR{h})(t).LTOE(i,1)*a;
                %             c=(Mat.Trajectory.(LR{h})(t).RTOE(i,2)-(Mat.Trajectory.(LR{h})(t).RANK(i,2)+Mat.Trajectory.(LR{h})(t).RANK_MED(i,2))/2)/...
                %                 (Mat.Trajectory.(LR{h})(t).RTOE(i,1)-(Mat.Trajectory.(LR{h})(t).RANK(i,1)+Mat.Trajectory.(LR{h})(t).RANK_MED(i,1))/2);
                %             d=Mat.Trajectory.(LR{h})(t).RTOE(i,2)-Mat.Trajectory.(LR{h})(t).RTOE(i,1)*a;
                %             if a==c
                %                 [x1, y1]=Mat.Trajectory.(LR{h})(t).LTOE(i,1:2);
                %                 [x2, y2]=(Mat.Trajectory.(LR{h})(t).LANK(i,1:2)+Mat.Trajectory.(LR{h})(t).LANK_MED(i,1:2))/2;
                %             else
                %             ML=(Mat.Trajectory.(LR{h})(t).LTOE(i,1:2)+(Mat.Trajectory.(LR{h})(t).LANK(i,1:2)+Mat.Trajectory.(LR{h})(t).LANK_MED(i,1:2))/2)/2;
                %             MR=(Mat.Trajectory.(LR{h})(t).RTOE(i,1:2)+(Mat.Trajectory.(LR{h})(t).RANK(i,1:2)+Mat.Trajectory.(LR{h})(t).RANK_MED(i,1:2))/2)/2;
                %             e=ML(2)+ML(1)/a;
                %             f=MR(2)+MR(1)/c;
                %
                %             x1=(d-b)/(a-c);
                %             y1=a*x1+b;
                %
                %             x2=(e-f)/(1/a-1/c);
                %             y2=-1/a*x2+e;
                %             end
                %             x=[x2 y2 0]-[x1 y1 0];
                %             z=[0 0 1];
                %             y=cross(z,x);
                
                Mat.Prog.(LR{h})(t).Org(i,:)=[0 0 0];
                Mat.Prog.(LR{h})(t).x(i,:)=[1 0 0];%x/norm(x);
                Mat.Prog.(LR{h})(t).y(i,:)=[0 1 0];%y/norm(y);
                Mat.Prog.(LR{h})(t).z(i,:)=[0 0 1];%z/norm(z);
            end
            
            % Pelvic Coordination______________________________________________________
            for i=1:Frames %Number Of Frames
                Mat.PelCo.(LR{h})(t).Org(i,:)=(Mat.Trajectory.(LR{h})(t).RASI(i,:)+Mat.Trajectory.(LR{h})(t).LASI(i,:))/2;
                MP(i,:)=(Mat.Trajectory.(LR{h})(t).RPSI(i,:)+Mat.Trajectory.(LR{h})(t).LPSI(i,:))/2;
                y=Mat.Trajectory.(LR{h})(t).LASI(i,:)-Mat.Trajectory.(LR{h})(t).RASI(i,:);
                z=cross(Mat.Trajectory.(LR{h})(t).LASI(i,:)-MP(i,:),y);
                x=cross(y,z);
                
                Mat.PelCo.(LR{h})(t).x(i,:)=x/norm(x);        % Obliquity
                Mat.PelCo.(LR{h})(t).y(i,:)=y/norm(y);        % Tilt
                Mat.PelCo.(LR{h})(t).z(i,:)=z/norm(z);        % Rotation
                
                clear x y z
            end
            clear MP
            
            % Hip Joint Center_________________________________________________________
            mm=MarkerDiameter/2;      %Marker Radius
            theta=0.5;
            beta=0.314;
            
            for i=1:Frames %Number Of Frames
                LegL(i,:)=norm(Mat.Trajectory.(LR{h})(t).LASI(i,:)-Mat.Trajectory.(LR{h})(t).LANK_MED(i,:));
                LegR(i,:)=norm(Mat.Trajectory.(LR{h})(t).RASI(i,:)-Mat.Trajectory.(LR{h})(t).RANK_MED(i,:));
                AsisD(i,:)=norm(Mat.Trajectory.(LR{h})(t).LASI(i,:)-Mat.Trajectory.(LR{h})(t).RASI(i,:));
                
                AsisTrocDistL(i,:)=0.1288*LegL(i,:)-48.56;
                AsisTrocDistR(i,:)=0.1288*LegR(i,:)-48.56;
                C(i,:)=(LegL(i,:)+LegR(i,:))/2*0.115-15.3;
                aa(i,:)=AsisD(i,:)/2;
                
                Lhjc(i,:)=[C(i,:)*cos(theta)*sin(beta)-(AsisTrocDistL(i,:)+mm)*cos(beta) ...
                    -(C(i,:)*sin(theta)-aa(i,:)) ...
                    -C(i,:)*cos(theta)*cos(beta)-(AsisTrocDistL(i,:)+mm)*sin(beta)];
                
                Rhjc(i,:)=[C(i,:)*cos(theta)*sin(beta)-(AsisTrocDistR(i,:)+mm)*cos(beta) ...
                    (C(i,:)*sin(theta)-aa(i,:)) ...
                    -C(i,:)*cos(theta)*cos(beta)-(AsisTrocDistR(i,:)+mm)*sin(beta)];
                
                Rot_Pel=[Mat.PelCo.(LR{h})(t).x(i,:)' Mat.PelCo.(LR{h})(t).y(i,:)' Mat.PelCo.(LR{h})(t).z(i,:)'];
                Mat.HJC_GL.(LR{h})(t).Left(i,:)=(Mat.PelCo.(LR{h})(t).Org(i,:)'+Rot_Pel*Lhjc(i,:)')';
                Mat.HJC_GL.(LR{h})(t).Right(i,:)=(Mat.PelCo.(LR{h})(t).Org(i,:)'+Rot_Pel*Rhjc(i,:)')';
            end
            clear LegL LegR AsisD AsisTrocDistL AsisTrocDistR C aa Lhjc Rhjc
            
            % Thigh Coordinate_________________________________________________________
            for i=1:Frames %Number Of Frames
                if h==1
                    z=Mat.HJC_GL.(LR{h})(t).Left(i,:)-(Mat.Trajectory.(LR{h})(t).LKNE(i,:)+Mat.Trajectory.(LR{h})(t).LKNE_MED(i,:))/2;
                    x=cross(Mat.Trajectory.(LR{h})(t).LKNE(i,:)-Mat.Trajectory.(LR{h})(t).LKNE_MED(i,:),z);
                    y=cross(z,x);
                    
                    Mat.ThiCo.(LR{h})(t).Org(i,:)=(Mat.Trajectory.(LR{h})(t).LKNE(i,:)+Mat.Trajectory.(LR{h})(t).LKNE_MED(i,:))/2;
                    Mat.ThiCo.(LR{h})(t).x(i,:)=x/norm(x);
                    Mat.ThiCo.(LR{h})(t).y(i,:)=y/norm(y);
                    Mat.ThiCo.(LR{h})(t).z(i,:)=z/norm(z);
                    clear x y z
                else
                    z=Mat.HJC_GL.(LR{h})(t).Right(i,:)-(Mat.Trajectory.(LR{h})(t).RKNE(i,:)+Mat.Trajectory.(LR{h})(t).RKNE_MED(i,:))/2;
                    x=cross(z,Mat.Trajectory.(LR{h})(t).RKNE(i,:)-Mat.Trajectory.(LR{h})(t).RKNE_MED(i,:));
                    y=cross(z,x);
                    
                    Mat.ThiCo.(LR{h})(t).Org(i,:)=(Mat.Trajectory.(LR{h})(t).RKNE(i,:)+Mat.Trajectory.(LR{h})(t).RKNE_MED(i,:))/2;
                    Mat.ThiCo.(LR{h})(t).x(i,:)=x/norm(x);
                    Mat.ThiCo.(LR{h})(t).y(i,:)=y/norm(y);
                    Mat.ThiCo.(LR{h})(t).z(i,:)=z/norm(z);
                    clear x y z
                end
            end
            
            
            % Shank Coordination_______________________________________________________
            for i=1:Frames%Number Of Frames
                % Tortioned Tibia for ankle angles
                if h==1
                    z=(Mat.Trajectory.(LR{h})(t).LKNE(i,:)+Mat.Trajectory.(LR{h})(t).LKNE_MED(i,:))/2-(Mat.Trajectory.(LR{h})(t).LANK(i,:)+Mat.Trajectory.(LR{h})(t).LANK_MED(i,:))/2;
                    x=cross(Mat.Trajectory.(LR{h})(t).LANK(i,:)-Mat.Trajectory.(LR{h})(t).LANK_MED(i,:),z);
                    y=cross(z,x);
                    
                    Mat.TibCo.(LR{h})(t).Org(i,:)=(Mat.Trajectory.(LR{h})(t).LANK(i,:)+Mat.Trajectory.(LR{h})(t).LANK_MED(i,:))/2;
                    Mat.TibCo.(LR{h})(t).x(i,:)=x/norm(x);
                    Mat.TibCo.(LR{h})(t).y(i,:)=y/norm(y);
                    Mat.TibCo.(LR{h})(t).z(i,:)=z/norm(z);
                    clear x y z
                else
                    z=(Mat.Trajectory.(LR{h})(t).RKNE(i,:)+Mat.Trajectory.(LR{h})(t).RKNE_MED(i,:))/2-(Mat.Trajectory.(LR{h})(t).RANK(i,:)+Mat.Trajectory.(LR{h})(t).RANK_MED(i,:))/2;
                    x=cross(z,Mat.Trajectory.(LR{h})(t).RANK(i,:)-Mat.Trajectory.(LR{h})(t).RANK_MED(i,:));
                    y=cross(z,x);
                    
                    Mat.TibCo.(LR{h})(t).Org(i,:)=(Mat.Trajectory.(LR{h})(t).RANK(i,:)+Mat.Trajectory.(LR{h})(t).RANK_MED(i,:))/2;
                    Mat.TibCo.(LR{h})(t).x(i,:)=x/norm(x);
                    Mat.TibCo.(LR{h})(t).y(i,:)=y/norm(y);
                    Mat.TibCo.(LR{h})(t).z(i,:)=z/norm(z);
                    clear x y z
                end
                
                
                % UnTortioned Tibia for knee angles
                if h==1
                    z=(Mat.Trajectory.(LR{h})(t).LKNE(i,:)+Mat.Trajectory.(LR{h})(t).LKNE_MED(i,:))/2-(Mat.Trajectory.(LR{h})(t).LANK(i,:)+Mat.Trajectory.(LR{h})(t).LANK_MED(i,:))/2;
                    x=cross(Mat.Trajectory.(LR{h})(t).LKNE(i,:)-Mat.Trajectory.(LR{h})(t).LKNE_MED(i,:),z);
                    y=cross(z,x);
                    
                    Mat.UNTibCo.(LR{h})(t).Org(i,:)=(Mat.Trajectory.(LR{h})(t).LANK(i,:)+Mat.Trajectory.(LR{h})(t).LANK_MED(i,:))/2;
                    Mat.UNTibCo.(LR{h})(t).x(i,:)=x/norm(x);
                    Mat.UNTibCo.(LR{h})(t).y(i,:)=y/norm(y);
                    Mat.UNTibCo.(LR{h})(t).z(i,:)=z/norm(z);
                    clear x y z
                else
                    z=(Mat.Trajectory.(LR{h})(t).RKNE(i,:)+Mat.Trajectory.(LR{h})(t).RKNE_MED(i,:))/2-(Mat.Trajectory.(LR{h})(t).RANK(i,:)+Mat.Trajectory.(LR{h})(t).RANK_MED(i,:))/2;
                    x=cross(z,Mat.Trajectory.(LR{h})(t).RKNE(i,:)-Mat.Trajectory.(LR{h})(t).RKNE_MED(i,:));
                    y=cross(z,x);
                    Mat.UNTibCo.(LR{h})(t).Org(i,:)=(Mat.Trajectory.(LR{h})(t).RANK(i,:)+Mat.Trajectory.(LR{h})(t).RANK_MED(i,:))/2;
                    Mat.UNTibCo.(LR{h})(t).x(i,:)=x/norm(x);
                    Mat.UNTibCo.(LR{h})(t).y(i,:)=y/norm(y);
                    Mat.UNTibCo.(LR{h})(t).z(i,:)=z/norm(z);
                    clear x y z
                end
                %             Mat.UNTibCo= Mat.TibCo;
            end
            
            
            % Foot Coordination________________________________________________________
            for i=1:Frames%Number Of Frames
                % The Main Foot Segment
                if h==1                    
%                     z=Mat.Trajectory.(LR{h})(t).LTOE(i,:)-Mat.Trajectory.(LR{h})(t).LHEE(i,:);
%                     x=cross(Mat.Trajectory.(LR{h})(t).LANK(i,:)-Mat.Trajectory.(LR{h})(t).LHEE(i,:),z);
%                     y=cross(z,x);

                    z=Mat.Trajectory.(LR{h})(t).LTOE(i,:)-Mat.Trajectory.(LR{h})(t).LHEE(i,:);
                    x=cross(Mat.Trajectory.(LR{h})(t).LANK(i,:)-Mat.Trajectory.(LR{h})(t).LANK_MED(i,:),z);
                    y=cross(z,x);
                    
                    Mat.FootCo.(LR{h})(t).Org(i,:)=Mat.Trajectory.(LR{h})(t).LHEE(i,:);%(Mat.Trajectory.(LR{h})(t).LANK(i,:)+Mat.Trajectory.(LR{h})(t).LANK_MED(i,:))/2;
                    Mat.FootCo.(LR{h})(t).x(i,:)=x/norm(x);
                    Mat.FootCo.(LR{h})(t).y(i,:)=y/norm(y);
                    Mat.FootCo.(LR{h})(t).z(i,:)=z/norm(z);
                    clear x y z
                else
                    
%                     z=Mat.Trajectory.(LR{h})(t).RTOE(i,:)-Mat.Trajectory.(LR{h})(t).RHEE(i,:);
%                     x=cross(z,Mat.Trajectory.(LR{h})(t).RANK(i,:)-Mat.Trajectory.(LR{h})(t).RHEE(i,:));
%                     y=cross(z,x);
                    
                    z=Mat.Trajectory.(LR{h})(t).RTOE(i,:)-Mat.Trajectory.(LR{h})(t).RANK_MED(i,:);
                    x=cross(z,Mat.Trajectory.(LR{h})(t).RANK(i,:)-Mat.Trajectory.(LR{h})(t).RHEE(i,:));
                    y=cross(z,x);
                    
                    Mat.FootCo.(LR{h})(t).Org(i,:)=Mat.Trajectory.(LR{h})(t).RHEE(i,:);%(Mat.Trajectory.(LR{h})(t).RANK(i,:)+Mat.Trajectory.(LR{h})(t).RANK_MED(i,:))/2;
                    Mat.FootCo.(LR{h})(t).x(i,:)=x/norm(x);
                    Mat.FootCo.(LR{h})(t).y(i,:)=y/norm(y);
                    Mat.FootCo.(LR{h})(t).z(i,:)=z/norm(z);
                    clear x y z
                end
            end
        end
    end
end

for h=1:length(LR)
    clear flag
    if h==1
        EssMRK=LEssMRK;
    else
        EssMRK=REssMRK;
    end
    for t=1:size(Mat.Trajectory.(LR{h}),2)
        Frames=size(Mat.Trajectory.(LR{h})(t).LASI,1);
        flag(t)=0;
        for o=1:length(EssMRK)
            if isfield(Mat.Trajectory.(LR{h})(t),(EssMRK{o}))
            if isempty(Mat.Trajectory.(LR{h})(t).(EssMRK{o}))
                flag(t)=1;
            end
            else
                flag(t)=1;
            end
        end
        if flag(t)==0
            for i=1:Frames
                %Local Pelvic Coordinate in Progression Frame CO_______________
                R_Prog=[Mat.Prog.(LR{h})(t).x(i,:)' Mat.Prog.(LR{h})(t).y(i,:)' Mat.Prog.(LR{h})(t).z(i,:)'];
                T=Mat.Prog.(LR{h})(t).Org(i,:);
                LASI_progco(i,:)=(R_Prog'*Mat.Trajectory.(LR{h})(t).LASI(i,:)'+T')';
                RASI_progco(i,:)=(R_Prog'*Mat.Trajectory.(LR{h})(t).RASI(i,:)'+T')';
                LPSI_progco(i,:)=(R_Prog'*Mat.Trajectory.(LR{h})(t).LPSI(i,:)'+T')';
                RPSI_progco(i,:)=(R_Prog'*Mat.Trajectory.(LR{h})(t).RPSI(i,:)'+T')';
                
                MP(i,:)=(RPSI_progco(i,:)+LPSI_progco(i,:))/2;
                
                y=LASI_progco(i,:)-RASI_progco(i,:);
                z=cross(LASI_progco(i,:)-MP(i,:),y);
                x=cross(y,z);
                LocalCo.PelCo.(LR{h})(t).Org(i,:)=(LASI_progco(i,:)+RASI_progco(i,:))/2;
                LocalCo.PelCo.(LR{h})(t).x(i,:)=x/norm(x);
                LocalCo.PelCo.(LR{h})(t).y(i,:)=y/norm(y);
                LocalCo.PelCo.(LR{h})(t).z(i,:)=z/norm(z);
                clear x y z
                
                %Local Thigh Coordinate in Pelvic Co_______________________________
                if h==1
                    R_Pel=[Mat.PelCo.(LR{h})(t).x(i,:)' Mat.PelCo.(LR{h})(t).y(i,:)' Mat.PelCo.(LR{h})(t).z(i,:)'];
                    T=Mat.PelCo.(LR{h})(t).Org(i,:);
                    LKNE_pelco(i,:)=(R_Pel'*(Mat.Trajectory.(LR{h})(t).LKNE(i,:)'-T'))';
                    LKNE_MED_pelco(i,:)=(R_Pel'*(Mat.Trajectory.(LR{h})(t).LKNE_MED(i,:)'-T'))';
                    LHJC_pelco(i,:)=(R_Pel'* (Mat.HJC_GL.(LR{h})(t).Left(i,:)'-T'))';
                    
                    z=LHJC_pelco(i,:)-(LKNE_pelco(i,:)+LKNE_MED_pelco(i,:))/2;
                    x=cross(LKNE_pelco(i,:)-LKNE_MED_pelco(i,:),z);
                    y=cross(z,x);
                    LocalCo.ThiCo.Left(t).Org(i,:)=(LKNE_pelco(i,:)+LKNE_MED_pelco(i,:))/2;
                    LocalCo.ThiCo.Left(t).x(i,:)=x/norm(x);
                    LocalCo.ThiCo.Left(t).y(i,:)=y/norm(y);
                    LocalCo.ThiCo.Left(t).z(i,:)=z/norm(z);
                    clear x y z
                end
                
                if h==2
                    R_Pel=[Mat.PelCo.(LR{h})(t).x(i,:)' Mat.PelCo.(LR{h})(t).y(i,:)' Mat.PelCo.(LR{h})(t).z(i,:)'];
                    T=Mat.PelCo.(LR{h})(t).Org(i,:);
                    RKNE_pelco(i,:)=(R_Pel'*(Mat.Trajectory.(LR{h})(t).RKNE(i,:)'-T'))';
                    RKNE_MED_pelco(i,:)=(R_Pel'*(Mat.Trajectory.(LR{h})(t).RKNE_MED(i,:)'-T'))';
                    RHJC_pelco(i,:)=(R_Pel'*(Mat.HJC_GL.(LR{h})(t).Right(i,:)'-T'))';
                    
                    z=RHJC_pelco(i,:)-(RKNE_pelco(i,:)+RKNE_MED_pelco(i,:))/2;
                    x=cross(z,RKNE_pelco(i,:)-RKNE_MED_pelco(i,:));
                    y=cross(z,x);
                    LocalCo.ThiCo.Right(t).Org(i,:)=(RKNE_pelco(i,:)+RKNE_MED_pelco(i,:))/2;
                    LocalCo.ThiCo.Right(t).x(i,:)=x/norm(x);
                    LocalCo.ThiCo.Right(t).y(i,:)=y/norm(y);
                    LocalCo.ThiCo.Right(t).z(i,:)=z/norm(z);
                    clear x y z R_Pel
                end
                
                %Local Torsioned Tibia Coordinate in Thigh Co_______________________________
                if h==1
                    R_THI=[Mat.ThiCo.Left(t).x(i,:)' Mat.ThiCo.Left(t).y(i,:)' Mat.ThiCo.Left(t).z(i,:)'];
                    T=Mat.ThiCo.Left(t).Org(i,:);
                    LKNE_thico(i,:)=(R_THI'*(Mat.Trajectory.(LR{h})(t).LKNE(i,:)'-T'))';
                    LKNE_MED_thico(i,:)=(R_THI'*(Mat.Trajectory.(LR{h})(t).LKNE_MED(i,:)'-T'))';
                    LANK_thico(i,:)=(R_THI'*(Mat.Trajectory.(LR{h})(t).LANK(i,:)'-T'))';
                    LANK_MED_thico(i,:)=(R_THI'*(Mat.Trajectory.(LR{h})(t).LANK_MED(i,:)'-T'))';
                    
                    z=(LKNE_thico(i,:)+LKNE_MED_thico(i,:))/2-(LANK_thico(i,:)+LANK_MED_thico(i,:))/2;
                    x=cross(LANK_thico(i,:)-LANK_MED_thico(i,:),z);
                    y=cross(z,x);
                    LocalCo.TibCo.Left(t).Org(i,:)=(LANK_thico(i,:)+LANK_MED_thico(i,:))/2;
                    LocalCo.TibCo.Left(t).x(i,:)=x/norm(x);
                    LocalCo.TibCo.Left(t).y(i,:)=y/norm(y);
                    LocalCo.TibCo.Left(t).z(i,:)=z/norm(z);
                    clear x y z
                end
                
                if h==2
                    R_THI=[Mat.ThiCo.Right(t).x(i,:)' Mat.ThiCo.Right(t).y(i,:)' Mat.ThiCo.Right(t).z(i,:)'];
                    T=Mat.ThiCo.Right(t).Org(i,:);
                    RKNE_thico(i,:)=(R_THI'*(Mat.Trajectory.(LR{h})(t).RKNE(i,:)'-T'))';
                    RKNE_MED_thico(i,:)=(R_THI'*(Mat.Trajectory.(LR{h})(t).RKNE_MED(i,:)'-T'))';
                    RANK_thico(i,:)=(R_THI'*(Mat.Trajectory.(LR{h})(t).RANK(i,:)'-T'))';
                    RANK_MED_thico(i,:)=(R_THI'*(Mat.Trajectory.(LR{h})(t).RANK_MED(i,:)'-T'))';
                    
                    z=(RKNE_thico(i,:)+RKNE_MED_thico(i,:))/2-(RANK_thico(i,:)+RANK_MED_thico(i,:))/2;
                    x=cross(z,RANK_thico(i,:)-RANK_MED_thico(i,:));
                    y=cross(z,x);
                    LocalCo.TibCo.Right(t).Org(i,:)=(RANK_thico(i,:)+RANK_MED_thico(i,:))/2;
                    LocalCo.TibCo.Right(t).x(i,:)=x/norm(x);
                    LocalCo.TibCo.Right(t).y(i,:)=y/norm(y);
                    LocalCo.TibCo.Right(t).z(i,:)=z/norm(z);
                    clear x y z
                end
                
                %Local UNTorsioned Tibia Coordinate in Thigh Co_______________________________
                if h==1
                    z=(LKNE_thico(i,:)+LKNE_MED_thico(i,:))/2-(LANK_thico(i,:)+LANK_MED_thico(i,:))/2;
                    x=cross(LKNE_thico(i,:)-LKNE_MED_thico(i,:),z);
                    y=cross(z,x);
                    
                    LocalCo.UNTibCo.Left(t).Org(i,:)=(LANK_thico(i,:)+LANK_MED_thico(i,:))/2;
                    LocalCo.UNTibCo.Left(t).x(i,:)=x/norm(x);
                    LocalCo.UNTibCo.Left(t).y(i,:)=y/norm(y);
                    LocalCo.UNTibCo.Left(t).z(i,:)=z/norm(z);
                    clear x y z
                end
                
                if h==2
                    z=(RKNE_thico(i,:)+RKNE_MED_thico(i,:))/2-(RANK_thico(i,:)+RANK_MED_thico(i,:))/2;
                    x=cross(z,RKNE_thico(i,:)-RKNE_MED_thico(i,:));
                    y=cross(z,x);
                    
                    LocalCo.UNTibCo.Right(t).Org(i,:)=(RANK_thico(i,:)+RANK_MED_thico(i,:))/2;
                    LocalCo.UNTibCo.Right(t).x(i,:)=x/norm(x);
                    LocalCo.UNTibCo.Right(t).y(i,:)=y/norm(y);
                    LocalCo.UNTibCo.Right(t).z(i,:)=z/norm(z);
                    clear x y z
                end
                %             LocalCo.UNTibCo=LocalCo.TibCo;
                
                %Local Foot Coordinate in Torsioned Tibia_______________________________
                if h==1
                    R_TIB=[Mat.TibCo.Left(t).x(i,:)' Mat.TibCo.Left(t).y(i,:)' Mat.TibCo.Left(t).z(i,:)'];
                    T=Mat.TibCo.Left(t).Org(i,:);
                    LTOE_tibco(i,:)=(R_TIB'*(Mat.Trajectory.(LR{h})(t).LTOE(i,:)'-T'))';
                    LANK_tibco(i,:)=(R_TIB'*(Mat.Trajectory.(LR{h})(t).LANK(i,:)'-T'))';
                    LANK_MED_tibco(i,:)=(R_TIB'*(Mat.Trajectory.(LR{h})(t).LANK_MED(i,:)'-T'))';
                    LHEE_tibco(i,:)=(R_TIB'*(Mat.Trajectory.(LR{h})(t).LHEE(i,:)'-T'))';
                    
%                     z=LTOE_tibco(i,:)-(LANK_tibco(i,:)+LANK_MED_tibco(i,:))/2;
                    z=LTOE_tibco(i,:)-LHEE_tibco(i,:);
%                     x=cross(LANK_tibco(i,:)-LHEE_tibco(i,:),z);
                    x=cross(LANK_tibco(i,:)-LANK_MED_tibco(i,:),z);
                    y=cross(z,x);
                    LocalCo.FootCo.Left(t).Org(i,:)=LHEE_tibco(i,:);%(LANK_tibco(i,:)+LANK_MED_tibco(i,:))/2;
                    LocalCo.FootCo.Left(t).x(i,:)=x/norm(x);
                    LocalCo.FootCo.Left(t).y(i,:)=y/norm(y);
                    LocalCo.FootCo.Left(t).z(i,:)=z/norm(z);
                    clear x y z
                end
                
                if h==2
                    R_TIB=[Mat.TibCo.Right(t).x(i,:)' Mat.TibCo.Right(t).y(i,:)' Mat.TibCo.Right(t).z(i,:)'];
                    T=Mat.TibCo.Right(t).Org(i,:);
                    RTOE_tibco(i,:)=(R_TIB'*(Mat.Trajectory.(LR{h})(t).RTOE(i,:)'-T'))';
                    RANK_tibco(i,:)=(R_TIB'*(Mat.Trajectory.(LR{h})(t).RANK(i,:)'-T'))';
                    RANK_MED_tibco(i,:)=(R_TIB'*(Mat.Trajectory.(LR{h})(t).RANK_MED(i,:)'-T'))';
                    RHEE_tibco(i,:)=(R_TIB'*(Mat.Trajectory.(LR{h})(t).RHEE(i,:)'-T'))';
                    
%                     z=RTOE_tibco(i,:)-(RANK_tibco(i,:)+RANK_MED_tibco(i,:))/2;
                    z=RTOE_tibco(i,:)-RHEE_tibco(i,:);
%                     x=cross(z,RANK_tibco(i,:)-RHEE_tibco(i,:));
                    x=cross(z,RANK_tibco(i,:)-RANK_MED_tibco(i,:));
                    y=cross(z,x);
                    LocalCo.FootCo.Right(t).Org(i,:)=RHEE_tibco(i,:);%(RANK_tibco(i,:)+RANK_MED_tibco(i,:))/2;
                    LocalCo.FootCo.Right(t).x(i,:)=x/norm(x);
                    LocalCo.FootCo.Right(t).y(i,:)=y/norm(y);
                    LocalCo.FootCo.Right(t).z(i,:)=z/norm(z);
                    clear x y z
                end
            end
        end
    end
end

Mat2=Mat;
end

