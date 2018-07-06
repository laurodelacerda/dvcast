
package dvcast;
import org.car2x.veins.modules.application.ieee80211p.BaseWaveApplLayer;

simple DVCastLayer extends BaseWaveApplLayer
{
    @class(DVCastLayer);
    string appName = default("DVCastLayer");
    double helloInterval = default(1s) @unit(s);
    bool sendHello = default(true); 
}