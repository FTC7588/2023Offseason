package org.firstinspires.ftc.teamcode.utils.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;

import org.opencv.core.Mat;

public class PixelCanvasAnnotator {

    private Mat cameraMatrix;

    float bmpPxToCanvasPx;
    float canvasDensityScale;

    Paint textPaint;
    Paint rectPaint;

    public PixelCanvasAnnotator(Mat cameraMatrix)
    {
        this.cameraMatrix = cameraMatrix;

        textPaint = new Paint();
        textPaint.setColor(Color.WHITE);
        textPaint.setAntiAlias(true);
        textPaint.setTypeface(Typeface.DEFAULT_BOLD);

        rectPaint = new Paint();
        rectPaint.setAntiAlias(true);
        rectPaint.setColor(Color.rgb(12, 145, 201));
        rectPaint.setStyle(Paint.Style.FILL);
    }

    public void noteDrawParams(float bmpPxToCanvasPx, float canvasDensityScale)
    {
        if (bmpPxToCanvasPx != this.bmpPxToCanvasPx || canvasDensityScale != this.canvasDensityScale)
        {
            this.bmpPxToCanvasPx = bmpPxToCanvasPx;
            this.canvasDensityScale = canvasDensityScale;

            textPaint.setTextSize(40*canvasDensityScale);
        }
    }

    public void drawCircle(float x, float y, float r, Canvas canvas) {
        canvas.drawCircle(x, y, r, textPaint);
    }

}
