function plot_foot_position(P1,P2,FF1,FF2,label1,label2,color1,color2,PERIOD)
% plot the left and right foot position
     
    figure

      t=(0:length(P1)-1)*PERIOD;
      plot(t,P1,'DisplayName',label1,'Color',color1,'LineWidth',1)
      hold on
      plot(t,P2,'DisplayName',label2,'Color',color2,'LineWidth',1)
      plot(FF1*PERIOD,P1(FF1),'o','DisplayName','Foot fall','Color',color1)      
      plot(FF2*PERIOD,P2(FF2),'o','DisplayName','Foot fall','Color',color2)
      
      xlabel('Time (s)')
      ylabel('Position (m)')
      legend('Location','northwest')
      
end