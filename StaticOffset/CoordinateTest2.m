function [Mat2, LocalCo] = CoordinateTest2(Mat,Static)
global MarkerDiameter

if isempty(MarkerDiameter)
Pr={ 'Marker Diameter (mm):'};
    Res = inputdlg(Pr,'Test Setting',[1 40],{ '9.5'});
    MarkerDiameter=str2num(Res{1});
end

LR={'Left' 'Right'};
LEssMRK={'LASI' 'RASI' 'LPSI' 'RPSI' 'LTHI' 'LTHI_B' 'LTHI_F' 'LTIB' 'LTIB_F' 'LTIB_B' 'LANK' 'LANK_MED' 'RANK_MED' 'LTOE' 'LHEE'};
REssMRK={'LASI' 'RASI' 'LPSI' 'RPSI' 'RTHI' 'RTHI_B' 'RTHI_F' 'RTIB' 'RTIB_F' 'RTIB_B' 'RANK' 'RANK_MED' 'LANK_MED' 'RTOE' 'RHEE'};

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
                    x=Mat.Trajectory.(LR{h})(t).LTHI_F(i,:)-Mat.Trajectory.(LR{h})(t).LTHI_B(i,:);
                    z=cross(x,Mat.Trajectory.(LR{h})(t).LTHI(i,:)-Mat.Trajectory.(LR{h})(t).LTHI_B(i,:));
                    y=cross(z,x);
                    
                    Mat.Thi2Co.(LR{h})(t).Org(i,:)=Mat.Trajectory.(LR{h})(t).LTHI(i,:);
                    Mat.Thi2Co.(LR{h})(t).x(i,:)=x/norm(x);
                    Mat.Thi2Co.(LR{h})(t).y(i,:)=y/norm(y);
                    Mat.Thi2Co.(LR{h})(t).z(i,:)=z/norm(z);
                    A=Static.HipRot.(LR{h})*[Mat.Thi2Co.(LR{h})(t).x(i,:);Mat.Thi2Co.(LR{h})(t).y(i,:);Mat.Thi2Co.(LR{h})(t).z(i,:)];
                    Mat.ThiCo.(LR{h})(t).org(i,:)=(Static.HipRot.(LR{h})*Mat.Thi2Co.(LR{h})(t).Org(i,:)')';
                    Mat.ThiCo.(LR{h})(t).x(i,:)=A(1,:);
                    Mat.ThiCo.(LR{h})(t).y(i,:)=A(2,:);
                    Mat.ThiCo.(LR{h})(t).z(i,:)=A(3,:);
                    clear A x y z
                else
                    x=Mat.Trajectory.(LR{h})(t).RTHI_F(i,:)-Mat.Trajectory.(LR{h})(t).RTHI_B(i,:);
                    z=cross(Mat.Trajectory.(LR{h})(t).RTHI(i,:)-Mat.Trajectory.(LR{h})(t).RTHI_B(i,:),x);
                    y=cross(z,x);
                    
                    Mat.Thi2Co.(LR{h})(t).Org(i,:)=Mat.Trajectory.(LR{h})(t).RTHI(i,:);
                    Mat.Thi2Co.(LR{h})(t).x(i,:)=x/norm(x);
                    Mat.Thi2Co.(LR{h})(t).y(i,:)=y/norm(y);
                    Mat.Thi2Co.(LR{h})(t).z(i,:)=z/norm(z);
                    A=Static.HipRot.Right*[Mat.Thi2Co.(LR{h})(t).x(i,:);Mat.Thi2Co.(LR{h})(t).y(i,:);Mat.Thi2Co.(LR{h})(t).z(i,:)];
                    Mat.ThiCo.(LR{h})(t).org(i,:)=(Static.HipRot.(LR{h})*Mat.Thi2Co.(LR{h})(t).Org(i,:)')';
                    Mat.ThiCo.(LR{h})(t).x(i,:)=A(1,:);
                    Mat.ThiCo.(LR{h})(t).y(i,:)=A(2,:);
                    Mat.ThiCo.(LR{h})(t).z(i,:)=A(3,:);
                    clear A x y z
                end
            end
            
            
            % Shank Coordination_______________________________________________________
            for i=1:Frames%Number Of Frames
                if h==1
                    x=Mat.Trajectory.(LR{h})(t).LTIB_F(i,:)-Mat.Trajectory.(LR{h})(t).LTIB_B(i,:);
                    z=cross(x,Mat.Trajectory.(LR{h})(t).LTIB_F(i,:)-Mat.Trajectory.(LR{h})(t).LTIB(i,:));
                    y=cross(z,x);
                    
                    Mat.Tib2Co.(LR{h})(t).Org(i,:)=Mat.Trajectory.(LR{h})(t).LTIB(i,:);
                    Mat.Tib2Co.(LR{h})(t).x(i,:)=x/norm(x);
                    Mat.Tib2Co.(LR{h})(t).y(i,:)=y/norm(y);
                    Mat.Tib2Co.(LR{h})(t).z(i,:)=z/norm(z);
                    A=Static.KneRot.Left*[Mat.Tib2Co.(LR{h})(t).x(i,:);Mat.Tib2Co.(LR{h})(t).y(i,:);Mat.Tib2Co.(LR{h})(t).z(i,:)];
                    Mat.UNTibCo.(LR{h})(t).org(i,:)=(Static.KneRot.(LR{h})*Mat.Tib2Co.(LR{h})(t).Org(i,:)')';
                    Mat.UNTibCo.(LR{h})(t).x(i,:)=A(1,:);
                    Mat.UNTibCo.(LR{h})(t).y(i,:)=A(2,:);
                    Mat.UNTibCo.(LR{h})(t).z(i,:)=A(3,:);
                    clear A
                    A=Static.AnkRot.Left*[Mat.Tib2Co.(LR{h})(t).x(i,:);Mat.Tib2Co.(LR{h})(t).y(i,:);Mat.Tib2Co.(LR{h})(t).z(i,:)];
                    Mat.TibCo.(LR{h})(t).org(i,:)=(Static.AnkRot.(LR{h})*Mat.Tib2Co.(LR{h})(t).Org(i,:)')';
                    Mat.TibCo.(LR{h})(t).x(i,:)=A(1,:);
                    Mat.TibCo.(LR{h})(t).y(i,:)=A(2,:);
                    Mat.TibCo.(LR{h})(t).z(i,:)=A(3,:);
                    clear A
                else
                    x=Mat.Trajectory.(LR{h})(t).RTIB_F(i,:)-Mat.Trajectory.(LR{h})(t).RTIB_B(i,:);
                    z=cross(Mat.Trajectory.(LR{h})(t).RTIB_F(i,:)-Mat.Trajectory.(LR{h})(t).RTIB(i,:),x);
                    y=cross(z,x);
                    
                    Mat.Tib2Co.(LR{h})(t).Org(i,:)=Mat.Trajectory.(LR{h})(t).RTIB(i,:);
                    Mat.Tib2Co.(LR{h})(t).x(i,:)=x/norm(x);
                    Mat.Tib2Co.(LR{h})(t).y(i,:)=y/norm(y);
                    Mat.Tib2Co.(LR{h})(t).z(i,:)=z/norm(z);
                    A=Static.KneRot.Right*[Mat.Tib2Co.(LR{h})(t).x(i,:);Mat.Tib2Co.(LR{h})(t).y(i,:);Mat.Tib2Co.(LR{h})(t).z(i,:)];
                    Mat.UNTibCo.(LR{h})(t).org(i,:)=(Static.KneRot.(LR{h})*Mat.Tib2Co.(LR{h})(t).Org(i,:)')';
                    Mat.UNTibCo.(LR{h})(t).x(i,:)=A(1,:);
                    Mat.UNTibCo.(LR{h})(t).y(i,:)=A(2,:);
                    Mat.UNTibCo.(LR{h})(t).z(i,:)=A(3,:);
                    clear A
                    A=Static.AnkRot.Right*[Mat.Tib2Co.(LR{h})(t).x(i,:);Mat.Tib2Co.(LR{h})(t).y(i,:);Mat.Tib2Co.(LR{h})(t).z(i,:)];
                    Mat.TibCo.(LR{h})(t).org(i,:)=(Static.AnkRot.(LR{h})*Mat.Tib2Co.(LR{h})(t).Org(i,:)')';
                    Mat.TibCo.(LR{h})(t).x(i,:)=A(1,:);
                    Mat.TibCo.(LR{h})(t).y(i,:)=A(2,:);
                    Mat.TibCo.(LR{h})(t).z(i,:)=A(3,:);
                    clear A
                end
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
                    
                    Mat.Foot2Co.(LR{h})(t).Org(i,:)=Mat.Trajectory.(LR{h})(t).LHEE(i,:);
                    Mat.Foot2Co.(LR{h})(t).x(i,:)=x/norm(x);
                    Mat.Foot2Co.(LR{h})(t).y(i,:)=y/norm(y);
                    Mat.Foot2Co.(LR{h})(t).z(i,:)=z/norm(z);
                    A=Static.FootRot.Left*[Mat.Foot2Co.(LR{h})(t).x(i,:);Mat.Foot2Co.(LR{h})(t).y(i,:);Mat.Foot2Co.(LR{h})(t).z(i,:)];
                    Mat.FootCo.(LR{h})(t).org(i,:)=(Static.FootRot.(LR{h})*Mat.Tib2Co.(LR{h})(t).Org(i,:)')';
                    Mat.FootCo.(LR{h})(t).x(i,:)=A(1,:);
                    Mat.FootCo.(LR{h})(t).y(i,:)=A(2,:);
                    Mat.FootCo.(LR{h})(t).z(i,:)=A(3,:);
                    clear x y z A
                else
%                     z=Mat.Trajectory.(LR{h})(t).RTOE(i,:)-Mat.Trajectory.(LR{h})(t).RHEE(i,:);
%                     x=cross(z,Mat.Trajectory.(LR{h})(t).RANK(i,:)-Mat.Trajectory.(LR{h})(t).RHEE(i,:));
%                     y=cross(z,x);
                    
                    z=Mat.Trajectory.(LR{h})(t).RTOE(i,:)-Mat.Trajectory.(LR{h})(t).RHEE(i,:);
                    x=cross(z,Mat.Trajectory.(LR{h})(t).RANK(i,:)-Mat.Trajectory.(LR{h})(t).RANK_MED(i,:));
                    y=cross(z,x);
                    
                    Mat.Foot2Co.(LR{h})(t).Org(i,:)=Mat.Trajectory.(LR{h})(t).RHEE(i,:);
                    Mat.Foot2Co.(LR{h})(t).x(i,:)=x/norm(x);
                    Mat.Foot2Co.(LR{h})(t).y(i,:)=y/norm(y);
                    Mat.Foot2Co.(LR{h})(t).z(i,:)=z/norm(z);
                    A=Static.FootRot.Right*[Mat.Foot2Co.(LR{h})(t).x(i,:);Mat.Foot2Co.(LR{h})(t).y(i,:);Mat.Foot2Co.(LR{h})(t).z(i,:)];
                    Mat.FootCo.(LR{h})(t).org(i,:)=(Static.FootRot.(LR{h})*Mat.Tib2Co.(LR{h})(t).Org(i,:)')';
                    Mat.FootCo.(LR{h})(t).x(i,:)=A(1,:);
                    Mat.FootCo.(LR{h})(t).y(i,:)=A(2,:);
                    Mat.FootCo.(LR{h})(t).z(i,:)=A(3,:);
                    clear x y z A
                end
            end
        end
    end
end
Mat2=Mat;



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
                LASI_progco(i,:)=(R_Prog'*(Mat.Trajectory.(LR{h})(t).LASI(i,:)'-T'))';
                RASI_progco(i,:)=(R_Prog'*(Mat.Trajectory.(LR{h})(t).RASI(i,:)'-T'))';
                LPSI_progco(i,:)=(R_Prog'*(Mat.Trajectory.(LR{h})(t).LPSI(i,:)'-T'))';
                RPSI_progco(i,:)=(R_Prog'*(Mat.Trajectory.(LR{h})(t).RPSI(i,:)'-T'))';
                
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
                R_Pel=[Mat.PelCo.(LR{h})(t).x(i,:)' Mat.PelCo.(LR{h})(t).y(i,:)' Mat.PelCo.(LR{h})(t).z(i,:)'];
                
                x=(R_Pel'*Mat.ThiCo.(LR{h})(t).x(i,:)')';
                y=(R_Pel'*Mat.ThiCo.(LR{h})(t).y(i,:)')';
                z=(R_Pel'*Mat.ThiCo.(LR{h})(t).z(i,:)')';
                
                LocalCo.ThiCo.(LR{h})(t).x(i,:)=x/norm(x);
                LocalCo.ThiCo.(LR{h})(t).y(i,:)=y/norm(y);
                LocalCo.ThiCo.(LR{h})(t).z(i,:)=z/norm(z);
                clear x y z
                
                
                %Local Torsioned Tibia Coordinate in Thigh Co_______________________________
                R_THI=[Mat.ThiCo.(LR{h})(t).x(i,:)' Mat.ThiCo.(LR{h})(t).y(i,:)' Mat.ThiCo.(LR{h})(t).z(i,:)'];
                
                x=(R_THI'*Mat.TibCo.(LR{h})(t).x(i,:)')';
                y=(R_THI'*Mat.TibCo.(LR{h})(t).y(i,:)')';
                z=(R_THI'*Mat.TibCo.(LR{h})(t).z(i,:)')';
                
                LocalCo.TibCo.(LR{h})(t).x(i,:)=x/norm(x);
                LocalCo.TibCo.(LR{h})(t).y(i,:)=y/norm(y);
                LocalCo.TibCo.(LR{h})(t).z(i,:)=z/norm(z);
                clear x y z
                
                %Local UNTorsioned Tibia Coordinate in Thigh Co_______________________________
                R_THI=[Mat.ThiCo.(LR{h})(t).x(i,:)' Mat.ThiCo.(LR{h})(t).y(i,:)' Mat.ThiCo.(LR{h})(t).z(i,:)'];
                
                x=(R_THI'*Mat.UNTibCo.(LR{h})(t).x(i,:)')';
                y=(R_THI'*Mat.UNTibCo.(LR{h})(t).y(i,:)')';
                z=(R_THI'*Mat.UNTibCo.(LR{h})(t).z(i,:)')';
                
                LocalCo.UNTibCo.(LR{h})(t).x(i,:)=x/norm(x);
                LocalCo.UNTibCo.(LR{h})(t).y(i,:)=y/norm(y);
                LocalCo.UNTibCo.(LR{h})(t).z(i,:)=z/norm(z);
                clear x y z
                
                %Local Foot Coordinate in Torsioned Tibia_______________________________
                R_TIB=[Mat.TibCo.(LR{h})(t).x(i,:)' Mat.TibCo.(LR{h})(t).y(i,:)' Mat.TibCo.(LR{h})(t).z(i,:)'];
                
                x=(R_TIB'*Mat.FootCo.(LR{h})(t).x(i,:)')';
                y=(R_TIB'*Mat.FootCo.(LR{h})(t).y(i,:)')';
                z=(R_TIB'*Mat.FootCo.(LR{h})(t).z(i,:)')';
                
                LocalCo.FootCo.(LR{h})(t).x(i,:)=x/norm(x);
                LocalCo.FootCo.(LR{h})(t).y(i,:)=y/norm(y);
                LocalCo.FootCo.(LR{h})(t).z(i,:)=z/norm(z);
                clear x y z
                
            end
        end
    end
end

end

