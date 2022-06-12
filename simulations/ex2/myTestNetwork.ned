package vanettutorials.simulations.ex2;

import org.car2x.veins.nodes.RSU;
import org.car2x.veins.nodes.Scenario;


network myTestNetwork extends Scenario
{
    @display("bgb=395,305");
    submodules:
        rsu[1]: RSU {
            @display("p=150,140;i=veins/sign/yellowdiamond;is=vs");
        }
}