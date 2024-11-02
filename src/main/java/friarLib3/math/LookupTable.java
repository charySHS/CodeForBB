package friarLib3.math;

import java.awt.*;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.concurrent.locks.Condition;

public class LookupTable 
{
    private ArrayList<Point2D.Double> Nodes = new ArrayList<Point2D.Double>();

    public LookupTable AddValue(double input, double output)
    {
        Nodes.add(new Point2D.Double(input, output));
        Nodes.sort(Comparator.comparingDouble(a -> a.x));

        return this;
    }

    public double GetValue(double input)
    {
        assert Nodes.size() > 1 : "LookupTable requires at least two values to lerp between";

        var numNodes = Nodes.size();

        // -- Clamp Low
        var low = Nodes.get(0);
        if (input <= low.x)
        {
            return low.y;
        }

        // -- Clamp High
        var high = Nodes.get(numNodes - 1);
        if (input >= high.x)
        {
            return high.y;
        }

        Point2D.Double Left = Nodes.get(0);
        Point2D.Double Right = null;

        // -- Find the node we're past
        for (Point2D.Double Node : Nodes)
        {
            if (input > Node.x)
            {
                Left = Node;
            }
            else
            {
                Right = Node;
                break;
            }
        }

        assert Right != null : "Didn't find a Right value";

        // -- Early out if the outputs are the same, make sure we don't divide by 0 
        if (Left.y == Right.y)
        {
            return Left.y;
        }

        return FriarMath.Remap(input, Left.x, Right.x, Left.y, Right.y);
    }




    
    
    static public class Normalized extends LookupTable
    {
        /**
         * A version of a lookup table where all inputs must be in the range of 0 to 1
         * When GetValue is called with a negative value, it will rotate the curve 180° around the origin such that
         * GetValue(0.5) returns 0.5, and GetValue(-0.5) returns -0.5
         * Automatically starts with values (0, 0) and (1, 1)
         */
        public Normalized()
        {
            AddValue(0, 0);
            AddValue(1, 1);
        }

        @Override
        public LookupTable AddValue(double input, double output)
        {
            assert input > 0 && input < 1 : "Normalized lookup tables must have their input in the range of 0 to 1";
            return super.AddValue(input, output);
        }

        @Override
        public double GetValue(double input)
        {
            var sign = input < 0 ? -1 : 1;
            return super.GetValue(Math.abs(input)) * sign;
        }
    }
}
