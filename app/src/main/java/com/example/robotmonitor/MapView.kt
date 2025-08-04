package com.example.robotmonitor

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View
import kotlin.apply
import kotlin.ranges.until

class MapView(context: Context, attrs: AttributeSet?) : View(context, attrs) {
    private var grid: Array<IntArray> = Array(GRID_SIZE) { IntArray(GRID_SIZE) }
    private var robotX: Int = 0
    private var robotY: Int = 0
    private var robotHeading: Int = 0
    private var isFinished: Boolean = false

    private val cellPaint = Paint().apply {
        style = Paint.Style.FILL
    }
    private val borderPaint = Paint().apply {
        style = Paint.Style.STROKE
        color = Color.BLACK
        strokeWidth = 1f
    }
    private val robotPaint = Paint().apply {
        color = Color.RED
        style = Paint.Style.FILL
    }
    private val textPaint = Paint().apply {
        color = Color.BLACK
        textSize = 20f
        textAlign = Paint.Align.CENTER
    }

    fun updateMap(grid: Array<IntArray>, robotX: Int, robotY: Int, robotHeading: Int, isFinished: Boolean) {
        this.grid = grid
        this.robotX = robotX
        this.robotY = robotY
        this.robotHeading = robotHeading
        this.isFinished = isFinished
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val cellSize = width.toFloat() / GRID_SIZE
        // Vẽ bản đồ: i là hàng (Y), j là cột (X)
        for (i in 0 until GRID_SIZE) {
            for (j in 0 until GRID_SIZE) {
                cellPaint.color = when {
                    i == robotY && j == robotX -> Color.GREEN // Robot
                    grid[j][i] == 2 -> Color.RED // Vật cản
                    grid[j][i] == 1 -> Color.BLUE // Ô đã thăm
                    isFinished && grid[j][i] == 0 -> Color.RED // Chưa thăm và chương trình kết thúc
                    else -> Color.WHITE // Chưa thăm (mặc định)
                }
                canvas.drawRect(
                    j * cellSize, i * cellSize,
                    (j + 1) * cellSize, (i + 1) * cellSize,
                    cellPaint
                )
                canvas.drawRect(
                    j * cellSize, i * cellSize,
                    (j + 1) * cellSize, (i + 1) * cellSize,
                    borderPaint
                )
            }
        }

        // Vẽ tọa độ X, Y
        for (i in 0 until GRID_SIZE) {
            canvas.drawText(i.toString(), (i + 0.5f) * cellSize, cellSize / 2, textPaint) // Trục X (ngang)
            canvas.drawText(i.toString(), cellSize / 2, (i + 0.5f) * cellSize, textPaint) // Trục Y (dọc)
        }

        // Vẽ robot: robotX ánh xạ vào trục X (ngang), robotY ánh xạ vào trục Y (dọc)
        val robotCenterX = (robotX + 0.5f) * cellSize // robotX là cột (X)
        val robotCenterY = (robotY + 0.5f) * cellSize // robotY là hàng (Y)
        canvas.drawCircle(robotCenterX, robotCenterY, cellSize / 4, robotPaint)

        // Vẽ mũi tên hướng
        val arrowLength = cellSize / 2
        val arrowEndX = robotCenterX + arrowLength * when (robotHeading) {
            90 -> 1f   // Hướng đông (X tăng)
            270 -> -1f // Hướng tây (X giảm)
            else -> 0f
        }
        val arrowEndY = robotCenterY + arrowLength * when (robotHeading) {
            0 -> -1f   // Hướng bắc (Y giảm)
            180 -> 1f  // Hướng nam (Y tăng)
            else -> 0f
        }
        canvas.drawLine(robotCenterX, robotCenterY, arrowEndX, arrowEndY, robotPaint)
    }

    companion object {
        const val GRID_SIZE = 10
    }
}