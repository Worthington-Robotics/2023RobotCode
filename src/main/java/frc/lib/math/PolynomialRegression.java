// NOTE: This file is available at http://algs4.cs.princeton.edu/14analysis/PolynomialRegression.java.html
package frc.lib.math;

/******************************************************************************
 *  Compilation:  javac -cp .:jama.jar PolynomialRegression.java
 *  Execution:    java  -cp .:jama.jar PolynomialRegression
 *  Dependencies: jama.jar StdOut.java
 *
 *  % java -cp .:jama.jar PolynomialRegression
 *  0.01 n^3 + -1.64 n^2 + 168.92 n + -2113.73 (R^2 = 0.997)
 *
 ******************************************************************************/

import Jama.Matrix;
import Jama.QRDecomposition;

/**
 * The {@code PolynomialRegression} class performs a polynomial regression on an set of <em>N</em> data points (
 * <em>y<sub>i</sub></em>, <em>x<sub>i</sub></em>). That is, it fits a polynomial <em>y</em> = &beta;<sub>0</sub> +
 * &beta;<sub>1</sub> <em>x</em> + &beta;<sub>2</sub> <em>x</em><sup>2</sup> + ... + &beta;<sub><em>d</em></sub>
 * <em>x</em><sup><em>d</em></sup> (where <em>y</em> is the response variable, <em>x</em> is the predictor variable, and
 * the &beta;<sub><em>i</em></sub> are the regression coefficients) that minimizes the sum of squared residuals of the
 * multiple regression model. It also computes associated the coefficient of determination <em>R</em><sup>2</sup>.
 * <p>
 * This implementation performs a QR-decomposition of the underlying Vandermonde matrix, so it is not the fastest or
 * most numerically stable way to perform the polynomial regression.
 *
 * @author Robert Sedgewick
 * @author Kevin Wayne
 */
public class PolynomialRegression {
    private int degree; // Degree of the polynomial regression
    private Matrix beta; // The polynomial regression coefficients
    private double sse; // Sum of squares due to error
    private double sst; // Total sum of squares

    public PolynomialRegression(double[][] xy, int degree) {
        double[] x = new double[xy.length];
        double[] y = new double[xy.length];
        for (int i = 0; i < xy.length; ++i) {
            x[i] = xy[i][0];
            y[i] = xy[i][1];
        }
        solve(x, y, degree);
    }

    /**
     * Performs a polynomial regression on the data points {@code (y[i], x[i])}.
     *
     * @param x      The values of the predictor variable
     * @param y      The corresponding values of the response variable
     * @param degree The degree of the polynomial to fit
     */
    public PolynomialRegression(double[] x, double[] y, int degree) {
        solve(x, y, degree);
    }

    private void solve(double[] x, double[] y, int degree) {
        this.degree = degree;

        int n = x.length;
        QRDecomposition qr = null;
        Matrix matrixX = null;

        // In case Vandermonde matrix does not have full rank, reduce degree until it does
        while (true) {

            // Build Vandermonde matrix
            double[][] vandermonde = new double[n][this.degree + 1];
            for (int i = 0; i < n; i++) {
                for (int j = 0; j <= this.degree; j++) {
                    vandermonde[i][j] = Math.pow(x[i], j);
                }
            }
            matrixX = new Matrix(vandermonde);

            // Find least squares solution
            qr = new QRDecomposition(matrixX);
            if (qr.isFullRank()) {
                break;
            }

            // Decrease degree and try again
            this.degree--;
        }

        // Create matrix from vector
        Matrix matrixY = new Matrix(y, n);

        // Linear regression coefficients
        beta = qr.solve(matrixY);

        // Mean of y[] values
        double sum = 0.0;
        for (int i = 0; i < n; i++) {
            sum += y[i];
        }
        double mean = sum / n;

        // Total variation to be accounted for
        for (int i = 0; i < n; i++) {
            double dev = y[i] - mean;
            sst += dev * dev;
        }

        // Variation not accounted for
        Matrix residuals = matrixX.times(beta).minus(matrixY);
        sse = residuals.norm2() * residuals.norm2();
    }

    /**
     * Returns the {@code j}th regression coefficient.
     *
     * @param j The index
     * @return The {@code j}th regression coefficient
     */
    public double beta(int j) {
        // to make -0.0 print as 0.0
        if (Math.abs(beta.get(j, 0)) < 1E-4) {
            return 0.0;
        }
        return beta.get(j, 0);
    }

    /**
     * Returns the degree of the polynomial to fit.
     *
     * @return The degree of the polynomial to fit
     */
    public int degree() {
        return degree;
    }

    /**
     * Returns the coefficient of determination <em>R</em><sup>2</sup>.
     *
     * @return The coefficient of determination <em>R</em><sup>2</sup>, which is a real number between 0 and 1
     */
    public double R2() {
        if (sst == 0.0) {
            return 1.0; // Constant function
        }
        return 1.0 - sse / sst;
    }

    /**
     * Returns the expected response {@code y} given the value of the predictor variable {@code x}.
     *
     * @param x The value of the predictor variable
     * @return The expected response {@code y} given the value of the predictor variable {@code x}
     */
    public double predict(double x) {
        // Horner's Method
        double y = 0.0;
        for (int j = degree; j >= 0; j--) {
            y = beta(j) + (x * y);
        }
        return y;
    }

    @Override
    public String toString() {
        StringBuilder s = new StringBuilder();
        int j = degree;

        // Ignoring leading zero coefficients
        while (j >= 0 && Math.abs(beta(j)) < 1E-5) {
            j--;
        }

        // Create remaining terms
        while (j >= 0) {
            if (j == 0) {
                s.append(String.format("%.2f ", beta(j)));
            } else if (j == 1) {
                s.append(String.format("%.2f x + ", beta(j)));
            } else {
                s.append(String.format("%.2f x^%d + ", beta(j), j));
            }
            j--;
        }
        s = s.append("  (R^2 = " + String.format("%.3f", R2()) + ")");
        return s.toString();
    }
}
