function [Mat2, LocalCo] = CoordinateStatic2(Mat)
    global MarkerDiameter
    if isempty(MarkerDiameter)
Pr={ 'Marker Diameter (mm):'};
    Res = inputdlg(Pr,'Test Setting',[1 40],{ '9.5'});
    MarkerDiameter=str2num(Res{1});
    end

    Frames=size(Mat.Trajectory.LASI,1);

    % Pelvic Coordination______________________________________________________
    for i=1:Frames %Number Of Frames
        Mat.PelCo.Org(i,:)=(Mat.Trajectory.RASI(i,:)+Mat.Trajectory.LASI(i,:))/2;
        MP(i,:)=(Mat.Trajectory.RPSI(i,:)+Mat.Trajectory.LPSI(i,:))/2;
        y=Mat.Trajectory.LASI(i,:)-Mat.Trajectory.RASI(i,:);
        z=cross(Mat.Trajectory.LASI(i,:)-MP(i,:),y);
        x=cross(y,z);

        Mat.PelCo.x(i,:)=x/norm(x);        % Obliquity
        Mat.PelCo.y(i,:)=y/norm(y);        % Tilt
        Mat.PelCo.z(i,:)=z/norm(z);        % Rotation

        clear x y z
    end


    % Hip Joint Center_________________________________________________________
    mm=MarkerDiameter/2;      %Marker Radius
    theta=0.5;
    beta=0.314;

    for i=1:Frames %Number Of Frames
        LegL(i,:)=norm(Mat.Trajectory.LASI(i,:)-Mat.Trajectory.LANK_MED(i,:));
        LegR(i,:)=norm(Mat.Trajectory.RASI(i,:)-Mat.Trajectory.RANK_MED(i,:));
        AsisD(i,:)=norm(Mat.Trajectory.LASI(i,:)-Mat.Trajectory.RASI(i,:));

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

        Rot_Pel=[Mat.PelCo.x(i,:)' Mat.PelCo.y(i,:)' Mat.PelCo.z(i,:)'];
        Mat.HJC_GL.Left(i,:)=(Mat.PelCo.Org(i,:)'+Rot_Pel*Lhjc(i,:)')';
        Mat.HJC_GL.Right(i,:)=(Mat.PelCo.Org(i,:)'+Rot_Pel*Rhjc(i,:)')';
    end


    % Thigh Coordinate_________________________________________________________
    for i=1:Frames %Number Of Frames
        z=Mat.HJC_GL.Left(i,:)-(Mat.Trajectory.LKNE(i,:)+Mat.Trajectory.LKNE_MED(i,:))/2;
        x=cross(Mat.Trajectory.LKNE(i,:)-Mat.Trajectory.LKNE_MED(i,:),z);
        y=cross(z,x);

        Mat.ThiCo.Left.Org(i,:)=(Mat.Trajectory.LKNE(i,:)+Mat.Trajectory.LKNE_MED(i,:))/2;
        Mat.ThiCo.Left.x(i,:)=x/norm(x);
        Mat.ThiCo.Left.y(i,:)=y/norm(y);
        Mat.ThiCo.Left.z(i,:)=z/norm(z);
        clear x y z

        z=Mat.HJC_GL.Right(i,:)-(Mat.Trajectory.RKNE(i,:)+Mat.Trajectory.RKNE_MED(i,:))/2;
        x=cross(z,Mat.Trajectory.RKNE(i,:)-Mat.Trajectory.RKNE_MED(i,:));
        y=cross(z,x);

        Mat.ThiCo.Right.Org(i,:)=(Mat.Trajectory.RKNE(i,:)+Mat.Trajectory.RKNE_MED(i,:))/2;
        Mat.ThiCo.Right.x(i,:)=x/norm(x);
        Mat.ThiCo.Right.y(i,:)=y/norm(y);
        Mat.ThiCo.Right.z(i,:)=z/norm(z);
        clear x y z
    end


    % Shank Coordination_______________________________________________________
    for i=1:Frames%Number Of Frames
       % Tortioned Tibia for ankle angles
         z=(Mat.Trajectory.LKNE(i,:)+Mat.Trajectory.LKNE_MED(i,:))/2-(Mat.Trajectory.LANK(i,:)+Mat.Trajectory.LANK_MED(i,:))/2;
        x=cross(Mat.Trajectory.LANK(i,:)-Mat.Trajectory.LANK_MED(i,:),z);
        y=cross(z,x);

        Mat.TibCo.Left.Org(i,:)=(Mat.Trajectory.LANK(i,:)+Mat.Trajectory.LANK_MED(i,:))/2;
        Mat.TibCo.Left.x(i,:)=x/norm(x);
        Mat.TibCo.Left.y(i,:)=y/norm(y);
        Mat.TibCo.Left.z(i,:)=z/norm(z);
        clear x y z

        z=(Mat.Trajectory.RKNE(i,:)+Mat.Trajectory.RKNE_MED(i,:))/2-(Mat.Trajectory.RANK(i,:)+Mat.Trajectory.RANK_MED(i,:))/2;
        x=cross(z,Mat.Trajectory.RANK(i,:)-Mat.Trajectory.RANK_MED(i,:));
        y=cross(z,x);

        Mat.TibCo.Right.Org(i,:)=(Mat.Trajectory.RANK(i,:)+Mat.Trajectory.RANK_MED(i,:))/2;
        Mat.TibCo.Right.x(i,:)=x/norm(x);
        Mat.TibCo.Right.y(i,:)=y/norm(y);
        Mat.TibCo.Right.z(i,:)=z/norm(z);
        clear x y z


        % UnTortioned Tibia for knee angles
        z=(Mat.Trajectory.LKNE(i,:)+Mat.Trajectory.LKNE_MED(i,:))/2-(Mat.Trajectory.LANK(i,:)+Mat.Trajectory.LANK_MED(i,:))/2;
        x=cross(Mat.Trajectory.LKNE(i,:)-Mat.Trajectory.LKNE_MED(i,:),z);
        y=cross(z,x);

        Mat.UNTibCo.Left.Org(i,:)=(Mat.Trajectory.LANK(i,:)+Mat.Trajectory.LANK_MED(i,:))/2;
        Mat.UNTibCo.Left.x(i,:)=x/norm(x);
        Mat.UNTibCo.Left.y(i,:)=y/norm(y);
        Mat.UNTibCo.Left.z(i,:)=z/norm(z);
        clear x y z

        z=(Mat.Trajectory.RKNE(i,:)+Mat.Trajectory.RKNE_MED(i,:))/2-(Mat.Trajectory.RANK(i,:)+Mat.Trajectory.RANK_MED(i,:))/2;
        x=cross(z,Mat.Trajectory.RKNE(i,:)-Mat.Trajectory.RKNE_MED(i,:));
        y=cross(z,x);

        Mat.UNTibCo.Right.Org(i,:)=(Mat.Trajectory.RANK(i,:)+Mat.Trajectory.RANK_MED(i,:))/2;
        Mat.UNTibCo.Right.x(i,:)=x/norm(x);
        Mat.UNTibCo.Right.y(i,:)=y/norm(y);
        Mat.UNTibCo.Right.z(i,:)=z/norm(z);
        clear x y z
    end


    % Foot Coordination________________________________________________________
    for i=1:Frames%Number Of Frames
        % The Main Foot Segment
%         z=Mat.Trajectory.LTOE(i,:)-Mat.Trajectory.LHEE(i,:);
%         x=cross(Mat.Trajectory.LANK(i,:)-Mat.Trajectory.LHEE(i,:),z);
%         y=cross(z,x);
        
        z=Mat.Trajectory.LTOE(i,:)-Mat.Trajectory.LHEE(i,:);
        x=cross(Mat.Trajectory.LANK(i,:)-Mat.Trajectory.LANK_MED(i,:),z);
        y=cross(z,x);
        
        Mat.FootCo.Left.Org(i,:)=Mat.Trajectory.LHEE(i,:);%(Mat.Trajectory.LANK(i,:)+Mat.Trajectory.LANK_MED(i,:))/2;
        Mat.FootCo.Left.x(i,:)=x/norm(x);
        Mat.FootCo.Left.y(i,:)=y/norm(y);
        Mat.FootCo.Left.z(i,:)=z/norm(z);
        clear x y z


%         z=Mat.Trajectory.RTOE(i,:)-Mat.Trajectory.RHEE(i,:);
%         x=cross(z,Mat.Trajectory.RANK(i,:)-Mat.Trajectory.RHEE(i,:));
%         y=cross(z,x);
        z=Mat.Trajectory.RTOE(i,:)-Mat.Trajectory.RHEE(i,:);
        x=cross(z,Mat.Trajectory.RANK(i,:)-Mat.Trajectory.RANK_MED(i,:));
        y=cross(z,x);
        
        Mat.FootCo.Right.Org(i,:)=Mat.Trajectory.RHEE(i,:);%(Mat.Trajectory.RANK(i,:)+Mat.Trajectory.RANK_MED(i,:))/2;
        Mat.FootCo.Right.x(i,:)=x/norm(x);
        Mat.FootCo.Right.y(i,:)=y/norm(y);
        Mat.FootCo.Right.z(i,:)=z/norm(z);
        clear x y z
    end
    
    % Thigh Coordination System-Second
for i=1:Frames
    x=Mat.Trajectory.LTHI_F(i,:)-Mat.Trajectory.LTHI_B(i,:);
    z=cross(x,Mat.Trajectory.LTHI(i,:)-Mat.Trajectory.LTHI_B(i,:));
    y=cross(z,x);
    
    Mat.Thi2Co.Left.Org(i,:)=Mat.Trajectory.LTHI(i,:);
    Mat.Thi2Co.Left.x(i,:)=x/norm(x);
    Mat.Thi2Co.Left.y(i,:)=y/norm(y);
    Mat.Thi2Co.Left.z(i,:)=z/norm(z);
    
    x=Mat.Trajectory.RTHI_F(i,:)-Mat.Trajectory.RTHI_B(i,:);
    z=cross(Mat.Trajectory.RTHI(i,:)-Mat.Trajectory.RTHI_B(i,:),x);
    y=cross(z,x);
    
    Mat.Thi2Co.Right.Org(i,:)=Mat.Trajectory.RTHI(i,:);
    Mat.Thi2Co.Right.x(i,:)=x/norm(x);
    Mat.Thi2Co.Right.y(i,:)=y/norm(y);
    Mat.Thi2Co.Right.z(i,:)=z/norm(z);
end


% Shank Coordination System-Second
for i=1:Frames

    x=Mat.Trajectory.LTIB_F(i,:)-Mat.Trajectory.LTIB_B(i,:);
    z=cross(x,Mat.Trajectory.LTIB_F(i,:)-Mat.Trajectory.LTIB(i,:));
    y=cross(z,x);
    
    Mat.Tib2Co.Left.Org(i,:)=Mat.Trajectory.LTIB(i,:);
    Mat.Tib2Co.Left.x(i,:)=x/norm(x);
    Mat.Tib2Co.Left.y(i,:)=y/norm(y);
    Mat.Tib2Co.Left.z(i,:)=z/norm(z);
    
    x=Mat.Trajectory.RTIB_F(i,:)-Mat.Trajectory.RTIB_B(i,:);
    z=cross(Mat.Trajectory.RTIB_F(i,:)-Mat.Trajectory.RTIB(i,:),x);
    y=cross(z,x);
    
    Mat.Tib2Co.Right.Org(i,:)=Mat.Trajectory.RTIB(i,:);
    Mat.Tib2Co.Right.x(i,:)=x/norm(x);
    Mat.Tib2Co.Right.y(i,:)=y/norm(y);
    Mat.Tib2Co.Right.z(i,:)=z/norm(z);
end

% Foot Coordination System-Second
for i=1:Frames

%     z=Mat.Trajectory.LTOE(i,:)-Mat.Trajectory.LHEE(i,:);
%     x=cross(Mat.Trajectory.LANK(i,:)-Mat.Trajectory.LHEE(i,:),z);
%     y=cross(z,x);
    z=Mat.Trajectory.LTOE(i,:)-Mat.Trajectory.LHEE(i,:);
    x=cross(Mat.Trajectory.LANK(i,:)-Mat.Trajectory.LANK_MED(i,:),z);
    y=cross(z,x);
    
    Mat.Foot2Co.Left.Org(i,:)=Mat.Trajectory.LHEE(i,:);
    Mat.Foot2Co.Left.x(i,:)=x/norm(x);
    Mat.Foot2Co.Left.y(i,:)=y/norm(y);
    Mat.Foot2Co.Left.z(i,:)=z/norm(z);
    

%     z=Mat.Trajectory.RTOE(i,:)-Mat.Trajectory.RHEE(i,:);
%     x=cross(z,Mat.Trajectory.RANK(i,:)-Mat.Trajectory.RHEE(i,:));
%     y=cross(z,x);
    z=Mat.Trajectory.RTOE(i,:)-Mat.Trajectory.RHEE(i,:);
    x=cross(z,Mat.Trajectory.RANK(i,:)-Mat.Trajectory.RANK_MED(i,:));
    y=cross(z,x);
    
    Mat.Foot2Co.Right.Org(i,:)=Mat.Trajectory.RHEE(i,:);
    Mat.Foot2Co.Right.x(i,:)=x/norm(x);
    Mat.Foot2Co.Right.y(i,:)=y/norm(y);
    Mat.Foot2Co.Right.z(i,:)=z/norm(z);
end

% Rotation Matrix for Hip Coordinates
dd=1;
x1=mean(Mat.ThiCo.Left.x(1:dd,:),1);
y1=mean(Mat.ThiCo.Left.y(1:dd,:),1);
z1=mean(Mat.ThiCo.Left.z(1:dd,:),1);
x2=mean(Mat.Thi2Co.Left.x(1:dd,:),1);
y2=mean(Mat.Thi2Co.Left.y(1:dd,:),1);
z2=mean(Mat.Thi2Co.Left.z(1:dd,:),1);
R=[x1;y1;z1]*[x2;y2;z2]';
Mat.HipRot.Left=R;
clear x1 y1 z1 x2 y2 z2 R
x1=mean(Mat.ThiCo.Right.x(1:dd,:),1);
y1=mean(Mat.ThiCo.Right.y(1:dd,:),1);
z1=mean(Mat.ThiCo.Right.z(1:dd,:),1);
x2=mean(Mat.Thi2Co.Right.x(1:dd,:),1);
y2=mean(Mat.Thi2Co.Right.y(1:dd,:),1);
z2=mean(Mat.Thi2Co.Right.z(1:dd,:),1);
R=[x1;y1;z1]*[x2;y2;z2]';
Mat.HipRot.Right=R;
clear x1 y1 z1 x2 y2 z2 R

% Rotation Matrix for Knee Coordinates
x1=mean(Mat.UNTibCo.Left.x(1:dd,:),1);
y1=mean(Mat.UNTibCo.Left.y(1:dd,:),1);
z1=mean(Mat.UNTibCo.Left.z(1:dd,:),1);
x2=mean(Mat.Tib2Co.Left.x(1:dd,:),1);
y2=mean(Mat.Tib2Co.Left.y(1:dd,:),1);
z2=mean(Mat.Tib2Co.Left.z(1:dd,:),1);
R=[x1;y1;z1]*[x2;y2;z2]';
Mat.KneRot.Left=R;
clear x1 y1 z1 x2 y2 z2 R

x1=mean(Mat.UNTibCo.Right.x(1:dd,:),1);
y1=mean(Mat.UNTibCo.Right.y(1:dd,:),1);
z1=mean(Mat.UNTibCo.Right.z(1:dd,:),1);
x2=mean(Mat.Tib2Co.Right.x(1:dd,:),1);
y2=mean(Mat.Tib2Co.Right.y(1:dd,:),1);
z2=mean(Mat.Tib2Co.Right.z(1:dd,:),1);
R=[x1;y1;z1]*[x2;y2;z2]';
Mat.KneRot.Right=R;
clear x1 y1 z1 x2 y2 z2 R

x1=mean(Mat.TibCo.Left.x(1:dd,:),1);
y1=mean(Mat.TibCo.Left.y(1:dd,:),1);
z1=mean(Mat.TibCo.Left.z(1:dd,:),1);
x2=mean(Mat.Tib2Co.Left.x(1:dd,:),1);
y2=mean(Mat.Tib2Co.Left.y(1:dd,:),1);
z2=mean(Mat.Tib2Co.Left.z(1:dd,:),1);
R=[x1;y1;z1]*[x2;y2;z2]';
Mat.AnkRot.Left=R;
clear x1 y1 z1 x2 y2 z2 R

x1=mean(Mat.TibCo.Right.x(1:dd,:),1);
y1=mean(Mat.TibCo.Right.y(1:dd,:),1);
z1=mean(Mat.TibCo.Right.z(1:dd,:),1);
x2=mean(Mat.Tib2Co.Right.x(1:dd,:),1);
y2=mean(Mat.Tib2Co.Right.y(1:dd,:),1);
z2=mean(Mat.Tib2Co.Right.z(1:dd,:),1);
R=[x1;y1;z1]*[x2;y2;z2]';
Mat.AnkRot.Right=R;
clear x1 y1 z1 x2 y2 z2 R


%% Rotation Matrix for Ankle Coordinates
x1=mean(Mat.FootCo.Left.x(1:dd,:),1);
y1=mean(Mat.FootCo.Left.y(1:dd,:),1);
z1=mean(Mat.FootCo.Left.z(1:dd,:),1);
x2=mean(Mat.Foot2Co.Left.x(1:dd,:),1);
y2=mean(Mat.Foot2Co.Left.y(1:dd,:),1);
z2=mean(Mat.Foot2Co.Left.z(1:dd,:),1);
R=[x1;y1;z1]*[x2;y2;z2]';
Mat.FootRot.Left=R;
clear x1 y1 z1 x2 y2 z2 R
x1=mean(Mat.FootCo.Right.x(1:dd,:),1);
y1=mean(Mat.FootCo.Right.y(1:dd,:),1);
z1=mean(Mat.FootCo.Right.z(1:dd,:),1);
x2=mean(Mat.Foot2Co.Right.x(1:dd,:),1);
y2=mean(Mat.Foot2Co.Right.y(1:dd,:),1);
z2=mean(Mat.Foot2Co.Right.z(1:dd,:),1);
R=[x1;y1;z1]*[x2;y2;z2]';
Mat.FootRot.Right=R;
clear x1 y1 z1 x2 y2 z2 R


    Mat2=Mat;
    
    
    
    for i=1:Frames
        %Local Thigh Coordinate in Pelvic Co_______________________________
        LocalCo.PelCo=Mat.PelCo;
        
        R_Pel=[Mat.PelCo.x(i,:)' Mat.PelCo.y(i,:)' Mat.PelCo.z(i,:)'];
        T=Mat.PelCo.Org(i,:);
        LKNE_pelco(i,:)=(R_Pel'*(Mat.Trajectory.LKNE(i,:)'-T'))';
        LKNE_MED_pelco(i,:)=(R_Pel'*(Mat.Trajectory.LKNE_MED(i,:)'-T'))';
        RKNE_pelco(i,:)=(R_Pel'*(Mat.Trajectory.RKNE(i,:)'-T'))';
        RKNE_MED_pelco(i,:)=(R_Pel'*(Mat.Trajectory.RKNE_MED(i,:)'-T'))';
        LHJC_pelco(i,:)=(R_Pel'*(Mat.HJC_GL.Left(i,:)'-T'))';
        RHJC_pelco(i,:)=(R_Pel'*(Mat.HJC_GL.Right(i,:)'-T'))';
             
        z=LHJC_pelco(i,:)-(LKNE_pelco(i,:)+LKNE_MED_pelco(i,:))/2;
        x=cross(LKNE_pelco(i,:)-LKNE_MED_pelco(i,:),z);
        y=cross(z,x);
        LocalCo.ThiCo.Left.Org(i,:)=(LKNE_pelco(i,:)+LKNE_MED_pelco(i,:))/2;
        LocalCo.ThiCo.Left.x(i,:)=x/norm(x);
        LocalCo.ThiCo.Left.y(i,:)=y/norm(y);
        LocalCo.ThiCo.Left.z(i,:)=z/norm(z);
        clear x y z

        z=RHJC_pelco(i,:)-(RKNE_pelco(i,:)+RKNE_MED_pelco(i,:))/2;
        x=cross(z,RKNE_pelco(i,:)-RKNE_MED_pelco(i,:));
        y=cross(z,x);
        LocalCo.ThiCo.Right.Org(i,:)=(RKNE_pelco(i,:)+RKNE_MED_pelco(i,:))/2;
        LocalCo.ThiCo.Right.x(i,:)=x/norm(x);
        LocalCo.ThiCo.Right.y(i,:)=y/norm(y);
        LocalCo.ThiCo.Right.z(i,:)=z/norm(z);
        clear x y z R_Pel
        
        
        %Local Torsioned Tibia Coordinate in Thigh Co_______________________________        
        R_THI=[Mat.ThiCo.Left.x(i,:)' Mat.ThiCo.Left.y(i,:)' Mat.ThiCo.Left.z(i,:)'];
        T=Mat.ThiCo.Left.Org(i,:);
        LKNE_thico(i,:)=(R_THI'*(Mat.Trajectory.LKNE(i,:)'-T'))';
        LKNE_MED_thico(i,:)=(R_THI'*(Mat.Trajectory.LKNE_MED(i,:)'-T'))';
        LANK_thico(i,:)=(R_THI'*(Mat.Trajectory.LANK(i,:)'-T'))';
        LANK_MED_thico(i,:)=(R_THI'*(Mat.Trajectory.LANK_MED(i,:)'-T'))';

        z=(LKNE_thico(i,:)+LKNE_MED_thico(i,:))/2-(LANK_thico(i,:)+LANK_MED_thico(i,:))/2;
        x=cross(LANK_thico(i,:)-LANK_MED_thico(i,:),z);
        y=cross(z,x);
        LocalCo.TibCo.Left.Org(i,:)=(LANK_thico(i,:)+LANK_MED_thico(i,:))/2;
        LocalCo.TibCo.Left.x(i,:)=x/norm(x);
        LocalCo.TibCo.Left.y(i,:)=y/norm(y);
        LocalCo.TibCo.Left.z(i,:)=z/norm(z);
        clear x y z
        
        R_THI=[Mat.ThiCo.Right.x(i,:)' Mat.ThiCo.Right.y(i,:)' Mat.ThiCo.Right.z(i,:)'];
        T=Mat.ThiCo.Right.Org(i,:);
        RKNE_thico(i,:)=(R_THI'*(Mat.Trajectory.RKNE(i,:)'-T'))';
        RKNE_MED_thico(i,:)=(R_THI'*(Mat.Trajectory.RKNE_MED(i,:)'-T'))';
        RANK_thico(i,:)=(R_THI'*(Mat.Trajectory.RANK(i,:)'-T'))';
        RANK_MED_thico(i,:)=(R_THI'*(Mat.Trajectory.RANK_MED(i,:)'-T'))';
        
        z=(RKNE_thico(i,:)+RKNE_MED_thico(i,:))/2-(RANK_thico(i,:)+RANK_MED_thico(i,:))/2;
        x=cross(z,RANK_thico(i,:)-RANK_MED_thico(i,:));
        y=cross(z,x);
        LocalCo.TibCo.Right.Org(i,:)=(RANK_thico(i,:)+RANK_MED_thico(i,:))/2;
        LocalCo.TibCo.Right.x(i,:)=x/norm(x);
        LocalCo.TibCo.Right.y(i,:)=y/norm(y);
        LocalCo.TibCo.Right.z(i,:)=z/norm(z);
        clear x y z
        
        
        %Local UNTorsioned Tibia Coordinate in Thigh Co_______________________________        
        z=(LKNE_thico(i,:)+LKNE_MED_thico(i,:))/2-(LANK_thico(i,:)+LANK_MED_thico(i,:))/2;
        x=cross(LKNE_thico(i,:)-LKNE_MED_thico(i,:),z);
        y=cross(z,x);

        LocalCo.UNTibCo.Left.Org(i,:)=(LANK_thico(i,:)+LANK_MED_thico(i,:))/2;
        LocalCo.UNTibCo.Left.x(i,:)=x/norm(x);
        LocalCo.UNTibCo.Left.y(i,:)=y/norm(y);
        LocalCo.UNTibCo.Left.z(i,:)=z/norm(z);
        clear x y z

        z=(RKNE_thico(i,:)+RKNE_MED_thico(i,:))/2-(RANK_thico(i,:)+RANK_MED_thico(i,:))/2;
        x=cross(z,RKNE_thico(i,:)-RKNE_MED_thico(i,:));
        y=cross(z,x);

        LocalCo.UNTibCo.Right.Org(i,:)=(RANK_thico(i,:)+RANK_MED_thico(i,:))/2;
        LocalCo.UNTibCo.Right.x(i,:)=x/norm(x);
        LocalCo.UNTibCo.Right.y(i,:)=y/norm(y);
        LocalCo.UNTibCo.Right.z(i,:)=z/norm(z);
        clear x y z
        
        
        %Local Foot Coordinate in Torsioned Tibia_______________________________        
        R_TIB=[Mat.TibCo.Left.x(i,:)' Mat.TibCo.Left.y(i,:)' Mat.TibCo.Left.z(i,:)'];
        T=Mat.TibCo.Left.Org(i,:);
        LTOE_tibco(i,:)=(R_TIB'*(Mat.Trajectory.LTOE(i,:)'-T'))';
        LANK_tibco(i,:)=(R_TIB'*(Mat.Trajectory.LANK(i,:)'-T'))';
        LANK_MED_tibco(i,:)=(R_TIB'*(Mat.Trajectory.LANK_MED(i,:)'-T'))';
        LHEE_tibco(i,:)=(R_TIB'*(Mat.Trajectory.LHEE(i,:)'-T'))';
        
%         z=LTOE_tibco(i,:)-(LANK_tibco(i,:)+LANK_MED_tibco(i,:))/2;
        z=LTOE_tibco(i,:)-LHEE_tibco(i,:);
%         x=cross(LANK_tibco(i,:)-LHEE_tibco(i,:),z);
         x=cross(LANK_tibco(i,:)-LANK_MED_tibco(i,:),z);
        y=cross(z,x);
        LocalCo.FootCo.Left.Org(i,:)=LHEE_tibco(i,:);%(LANK_tibco(i,:)+LANK_MED_tibco(i,:))/2;
        LocalCo.FootCo.Left.x(i,:)=x/norm(x);
        LocalCo.FootCo.Left.y(i,:)=y/norm(y);
        LocalCo.FootCo.Left.z(i,:)=z/norm(z);
        clear x y z


        R_TIB=[Mat.TibCo.Right.x(i,:)' Mat.TibCo.Right.y(i,:)' Mat.TibCo.Right.z(i,:)'];
        T=Mat.TibCo.Right.Org(i,:);
        RTOE_tibco(i,:)=(R_TIB'*(Mat.Trajectory.RTOE(i,:)'-T'))';
        RANK_tibco(i,:)=(R_TIB'*(Mat.Trajectory.RANK(i,:)'-T'))';
        RANK_MED_tibco(i,:)=(R_TIB'*(Mat.Trajectory.RANK_MED(i,:)'-T'))';
         RHEE_tibco(i,:)=(R_TIB'*(Mat.Trajectory.RHEE(i,:)'-T'))';
 
%         z=RTOE_tibco(i,:)-(RANK_tibco(i,:)+RANK_MED_tibco(i,:))/2;
        z=RTOE_tibco(i,:)-RHEE_tibco(i,:);
%         x=cross(z,RANK_tibco(i,:)-RHEE_tibco(i,:));
        x=cross(z,RANK_tibco(i,:)-RANK_MED_tibco(i,:));
        y=cross(z,x);
        LocalCo.FootCo.Right.Org(i,:)=RHEE_tibco(i,:);%(RANK_tibco(i,:)+RANK_MED_tibco(i,:))/2;
        LocalCo.FootCo.Right.x(i,:)=x/norm(x);
        LocalCo.FootCo.Right.y(i,:)=y/norm(y);
        LocalCo.FootCo.Right.z(i,:)=z/norm(z);
        clear x y z
        
    end
end

