clc
shallstoredata = false;
if(~shallstoredata)
    clear
    ploterror = true;
    load('data/simSensor20220311rmsdata.mat')
    % Generate Q and R values
    iteration_count =500;
    b=0.02;
    for i=1:iteration_count
        p = 10^(b*(i-0.6*iteration_count)); %default p = 1
        % Measurement noise covariances
        kr(1,i)  = 200*p;
        kr_(1,i) = p;
        % Process noise covariance
        kq(1,i) = 0.1*p;
        kq_(1,i) = 0.01*p;
    end
    
    %%
    [row,col]=size(rollRms);
    for i=1:row
        for j=1:col
            productRms(i,j)=rollRms(i,j)*pitchRms(i,j)*yawRms(i,j);
        end
    end
    
    rRms = reshape(rollRms,[],1);
    pRms = reshape(pitchRms,[],1);
    yRms = reshape(yawRms,[],1);
    N = length(rRms);
    % weight
    W=[1 1 1];
    for n=1:N
        radius(n) = sqrt(W(1)*rRms(n)^2+W(2)*pRms(n)^2+W(3)*yRms(n)^2);
    end
    minRadius = min(radius);
    [minidx,minidy] = find(radius==minRadius);
    idx = mod(minidy,length(kq));
    idy = ceil(minidy/length(kq));
    for ix=1:length(idx)
        if(idx(ix)==0)
            idx(ix)=iteration_count;
        end
    end
    for iy=1:length(idy)
        minRms(1,iy) = rollRms(idx(iy),idy(iy));
        minRms(2,iy) = pitchRms(idx(iy),idy(iy));
        minRms(3,iy) = yawRms(idx(iy),idy(iy));
    end
    %     subplot(1,2,2)
    fig1 =figure('Name','RMS_coordinate');
    plot3(rRms,pRms,yRms,'-')
    hold on
    [X,Y,Z]=sphere;
    X1 = minRadius * X;
    Y1 = minRadius * Y;
    Z1 = minRadius * Z;
    mesh(X1,Y1,Z1);
    xlim([-2 40]);
    ylim([-2 40]);
    zlim([0 20]);
    xlabel('roll rms')
    ylabel('pitch rms')
    zlabel("yaw rms")
    title("rms error of roll, pitch and yaw coordinate")
    legend('radius of RMS','sphere with radius of minimum RMS radius');
    
    %
    atQR = [kq(1,idx);kr(1,idy)]
    save('data/kqkr220204.mat','atQR')
    
    fig2 = figure('Name','attitude_RMS');
    A=axes;
    mesh(kr,kq,rollRms,'EdgeColor',	'b')
    hold on
    mesh(kr,kq,pitchRms,'EdgeColor',	'r')
    mesh(kr,kq,yawRms,'EdgeColor',	'g')
    %     mesh(productRms,'EdgeColor',	'r')
    zlim([-0.1 10])
    plot3(kr(rollidy),kq(rollidx),rollrmsmin*ones(size(rollidx)),'o-','Color','#FF00FF',"LineWidth",4);
    plot3(kr(pitchidy),kq(pitchidx),pitchrmsmin*ones(size(pitchidx)),'*-','Color','#FFFF00',"LineWidth",4);
    plot3(kr(yawidy),kq(yawidx),yawrmsmin*ones(size(yawidx)),'+-','Color','#D95319',"LineWidth",4);
    plot3(kr(idy),kq(idx),rollRms(idx(1),idy(1))*ones(size(idx)),'o-','Color','#00FFFF',"LineWidth",4);
    plot3(kr(idy),kq(idx),pitchRms(idx(1),idy(1))*ones(size(idx)),'x-','Color','#000000',"LineWidth",4);
    plot3(kr(idy),kq(idx),yawRms(idx(1),idy(1))*ones(size(idx)),'w-','Color','#7E2F8E',"LineWidth",4);
    set(A,'YScale','log','XScale','log');
    xlabel('kr')
    ylabel('kq')
    zlabel("RMS (degree)")
    title("RMS Error of Roll, Pitch and Yaw")
    legend('Roll RMS','Pitch RMS','Yaw RMS','min Roll RMS','min Pitch RMS','min Yaw RMS','optimal Roll RMS','optimal Pitch RMS','optimal Yaw RMS');
    
    % plot(radius)
    %% Erro plot
    if ploterror
        t=linspace(0,30,6000);
        fig3 = figure('Name','Roll_ERR_varyingR');
        A=axes;
        %     subplot(2,2,1);
        mesh(t,kr,rollErrR);
        set(A,'YScale','log')
        xlabel('time (s)')
        ylabel('kr')
        zlabel("roll error")
        title("Roll  Error for varying R and constant Q")
        %     subplot(2,2,2);
        fig4 = figure('Name','Pitch_ERR_varyingR');
        A=axes;
        mesh(t,kr,pitchErrR);
        set(A,'YScale','log')
        xlabel('time (s)')
        ylabel('kr')
        zlabel("pitch error")
        title("pitch  Error for varying R and constant Q")
        %     subplot(2,2,3);
        fig5 = figure('Name','Yaw_ERR_varyingR');
        A=axes;
        mesh(t,kr,yawErrR);
        set(A,'YScale','log')
        xlabel('time (s)')
        ylabel('kr')
        zlabel("yaw error")
        title("yaw  Error for varying R and constant Q")
        fig6 = figure('Name','Roll_ERR_varyingQ');
        %     subplot(2,2,1);
        A=axes;
        mesh(t,kq,rollErrQ);
        set(A,'YScale','log')
        xlabel('time (s)')
        ylabel('kq')
        zlabel("roll error")
        title("Roll  Error for varying Q and constant R")
        %     subplot(2,2,2);
        fig7 = figure('Name','Pitch_ERR_varyingR');
        A=axes;
        mesh(t,kq,pitchErrQ);
        set(A,'YScale','log')
        xlabel('time (s)')
        ylabel('kq')
        zlabel("pitch error")
        title("Pitch  Error for varying Q and constant R")
        %     subplot(2,2,3);
        fig8 = figure('Name','Attitude_ERR_varyingR');
        A=axes;
        set(gca,'FontSize',12);
        mesh(t,kq,yawErrQ);
        set(A,'YScale','log')
        xlabel('time (s)')
        ylabel('kq')
        zlabel("yaw error")
        title("Yaw  Error for varying Q and constant R")
    end
else
    save('data/simSensor20220311rmsdata.mat','rollRms','pitchRms','yawRms','rollrmsmin','pitchrmsmin','yawrmsmin','pitchErrR','rollErrR','yawErrR','pitchErrQ','rollErrQ','yawErrQ','rollidx','rollidy','pitchidx','pitchidy','yawidx','yawidy');
end