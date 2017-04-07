import java.util.ArrayList;

class CorrectionResult{
	public double leftValue, rightValue, correctionFactor;
	public CorrectionResult(double correctionFactor, double leftValue, double rightValue){
		this.leftValue = leftValue;
		this.rightValue = rightValue;
		this.correctionFactor = correctionFactor;
	}
}

class CorrectionInput{
	public double mSpeed, leftDistance, rightDistance;
	public CorrectionInput(double mSpeed, double leftDistance, double rightDistance){
		this.mSpeed = mSpeed;
		this.leftDistance = leftDistance;
		this.rightDistance = rightDistance;
	}
}

public class Main {
	private static CorrectionResult errorCorrection(CorrectionInput input){
		double mSpeed = input.mSpeed;
		double leftDistance = input.leftDistance;
		double rightDistance = input.rightDistance;
	    double averageDistance = (leftDistance + rightDistance) / 2;	    
	    double error = 0;
		double correctionFactor = 0;
		double ERROR_SCALING_CONST_P = 0.04;
		double resultantleftspeed, resultantrightspeed;
		if(mSpeed < 0){
			
		    error = leftDistance - rightDistance; // if error > 0 left is ahead subtract error from left
		                                          // if error < 0 right is ahead add -error to right
		    correctionFactor = error * ERROR_SCALING_CONST_P; // dampen error
		
		    resultantleftspeed = mSpeed - correctionFactor;
		    resultantrightspeed = mSpeed + correctionFactor;
		    if (resultantleftspeed > 1.0)
		      resultantleftspeed = 1.0;
		    if (resultantrightspeed > 1.0)
		      resultantrightspeed = 1.0;
		    if (resultantleftspeed < -1.0)
		      resultantleftspeed = -1.0;
		    if (resultantrightspeed < -1.0)
		      resultantrightspeed = -1.0;
	    }
	    else{
	    	
		    error = rightDistance - leftDistance; // if error > 0 left is ahead subtract error from left
		                                          // if error < 0 right is ahead add -error to right
		    correctionFactor = error * ERROR_SCALING_CONST_P; // dampen error
		
		    resultantleftspeed = mSpeed - correctionFactor;
		    resultantrightspeed = mSpeed + correctionFactor;
		    if (resultantleftspeed > 1.0)
		      resultantleftspeed = 1.0;
		    if (resultantrightspeed > 1.0)
		      resultantrightspeed = 1.0;
		    if (resultantleftspeed < -1.0)
		      resultantleftspeed = -1.0;
		    if (resultantrightspeed < -1.0)
		      resultantrightspeed = -1.0;
	    }
		return new CorrectionResult(correctionFactor, resultantleftspeed, resultantrightspeed);
	}
	public static void main(String[] args){
		ArrayList<CorrectionInput> unitTest = new ArrayList<CorrectionInput>();
		unitTest.add(new CorrectionInput(0.25, -20, -25));
		unitTest.add(new CorrectionInput(0.25, -25, -20));
		unitTest.add(new CorrectionInput(-0.25, -20, -25));
		unitTest.add(new CorrectionInput(-0.25, -25, -20));
		for(CorrectionInput i: unitTest){
			CorrectionResult result = errorCorrection(i);
			System.out.println(result.correctionFactor + ", " + result.leftValue + ", " + result.rightValue);
		}
	}
}
