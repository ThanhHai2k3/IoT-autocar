package com.example.robotmonitor

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.ProgressBar
import android.widget.TextView
import androidx.activity.enableEdgeToEdge
import androidx.activity.viewModels
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.widget.NestedScrollView
import androidx.recyclerview.widget.DiffUtil
import androidx.recyclerview.widget.GridLayoutManager
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.google.android.material.switchmaterial.SwitchMaterial
import kotlin.getValue

class MainActivity : AppCompatActivity() {

    private lateinit var statusText: TextView
    private lateinit var wifiSpeedGrid: RecyclerView
    private lateinit var realtimeSwitch: SwitchMaterial
    private lateinit var stopButton: Button
    private lateinit var mapView: MapView
    private lateinit var positionText: TextView
    private lateinit var headingText: TextView
    private lateinit var distanceText: TextView
    private lateinit var directionText: TextView

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        // View Binding
        statusText = findViewById(R.id.statusText)
        wifiSpeedGrid = findViewById(R.id.wifiSpeedGrid)
        realtimeSwitch = findViewById(R.id.realtimeSwitch)
        stopButton = findViewById(R.id.stopButton)
        mapView = findViewById(R.id.mapView)
        positionText = findViewById(R.id.positionText)
        headingText = findViewById(R.id.headingText)
        distanceText = findViewById(R.id.distanceText)
        directionText = findViewById(R.id.directionText)
    }

    data class WifiSpeedCell(val row: Int, val col: Int, val speed: Double)

    inner class WifiSpeedAdapter : ListAdapter<WifiSpeedCell, WifiSpeedViewHolder>(WifiSpeedDiffCallback()) {
        private var maxSpeedX: Int = -1
        private var maxSpeedY: Int = -1

        fun setMaxSpeedPosition(x: Int, y: Int) {
            maxSpeedX = x
            maxSpeedY = y
            notifyDataSetChanged()
        }

        override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): WifiSpeedViewHolder {
            val view = LayoutInflater.from(parent.context).inflate(R.layout.item_wifi_speed, parent, false)
            return WifiSpeedViewHolder(view)
        }

        override fun onBindViewHolder(holder: WifiSpeedViewHolder, position: Int) {
            val cell = getItem(position)
            holder.bind(cell, cell.col == maxSpeedX && cell.row == maxSpeedY)
        }
    }

    inner class WifiSpeedViewHolder(itemView: View) : RecyclerView.ViewHolder(itemView) {
        private val speedText: TextView = itemView.findViewById(R.id.speedText)

        fun bind(cell: WifiSpeedCell, isMaxSpeed: Boolean) {
            speedText.text = if (cell.speed > -100) "${"%.1f".format(cell.speed)} dBm" else "-"
            speedText.setBackgroundColor(
                if (isMaxSpeed) android.graphics.Color.parseColor("#C8E6C9")
                else android.graphics.Color.WHITE
            )
        }
    }

    class WifiSpeedDiffCallback : DiffUtil.ItemCallback<WifiSpeedCell>() {
        override fun areItemsTheSame(oldItem: WifiSpeedCell, newItem: WifiSpeedCell): Boolean {
            return oldItem.row == newItem.row && oldItem.col == newItem.col
        }

        override fun areContentsTheSame(oldItem: WifiSpeedCell, newItem: WifiSpeedCell): Boolean {
            return oldItem.speed == newItem.speed
        }
    }
}
