package friarLib3.math;

public class FriarMath
{
    public static double Remap(double value, double inLow, double inHigh, double outLow, double outHigh)
    {
        return Remap(value, inLow, inHigh, outLow, outHigh, true);    
    }
    
    public static double Remap(double value, double inLow, double inHigh, double outLow, double outHigh, boolean clamp)
    {
        double output = outLow + (value - inLow) * (outHigh - outLow) / (inHigh - inLow);
        
        if (clamp)
        {
            return Math.min(Math.max(output, outLow), outHigh);
        }
        
        return output;
    }
    
    
}