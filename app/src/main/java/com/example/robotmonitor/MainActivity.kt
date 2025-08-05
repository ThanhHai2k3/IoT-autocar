package com.example.robotmonitor

import android.annotation.SuppressLint
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
import androidx.lifecycle.Observer
import androidx.recyclerview.widget.DiffUtil
import androidx.recyclerview.widget.GridLayoutManager
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.android.volley.Request.Method
import com.android.volley.toolbox.StringRequest
import com.android.volley.toolbox.Volley
import com.example.robotmonitor.RobotViewModel.RobotData
import com.google.android.material.switchmaterial.SwitchMaterial
import kotlin.collections.indices
import kotlin.getValue

class MainActivity : AppCompatActivity() {

    private val viewModel: RobotViewModel by viewModels()
    private lateinit var positionText: TextView
    private lateinit var headingText: TextView
    private lateinit var wifiSpeedText: TextView
    private lateinit var maxWifiSpeedText: TextView
    private lateinit var distanceText: TextView
    private lateinit var directionText: TextView
    private lateinit var statusText: TextView
    private lateinit var mapView: MapView
    private lateinit var progressBar: ProgressBar
    private lateinit var realtimeSwitch: SwitchMaterial
    private lateinit var wifiSpeedGrid: RecyclerView
    private lateinit var nestedScrollView: NestedScrollView
    private lateinit var stopButton: Button
    private lateinit var wifiSpeedAdapter: WifiSpeedAdapter
    private var lastWifiSpeeds: Array<DoubleArray>? = null
    private val serverIp = "172.20.10.2" // Thống nhất IP với RobotViewModel

    @SuppressLint("MissingInflatedId")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        // Khởi tạo các view
        positionText = findViewById(R.id.positionText)
        headingText = findViewById(R.id.headingText)
        wifiSpeedText = findViewById(R.id.wifiSpeedText)
        maxWifiSpeedText = findViewById(R.id.maxWifiSpeedText)
        distanceText = findViewById(R.id.distanceText)
        directionText = findViewById(R.id.directionText)
        statusText = findViewById(R.id.statusText)
        mapView = findViewById(R.id.mapView)
        progressBar = findViewById(R.id.progressBar)
        realtimeSwitch = findViewById(R.id.realtimeSwitch)
        wifiSpeedGrid = findViewById(R.id.wifiSpeedGrid)
        nestedScrollView = findViewById(R.id.nestedScrollView)
        stopButton = findViewById(R.id.stopButton)

        // Thiết lập RecyclerView cho bảng RSSI
        wifiSpeedAdapter = WifiSpeedAdapter()
        wifiSpeedGrid.layoutManager = GridLayoutManager(this, 10)
        wifiSpeedGrid.adapter = wifiSpeedAdapter

        // Quan sát LiveData
        viewModel.robotData.observe(this, Observer { data ->
            updateUI(data)
            stopButton.isEnabled = !data.isFinished
        })
        viewModel.error.observe(this, Observer { error ->
            val scrollY = nestedScrollView.scrollY
            statusText.text = error ?: "Status: Connected"
            statusText.setTextColor(
                if (error != null) getColor(android.R.color.holo_red_dark)
                else getColor(android.R.color.holo_green_dark)
            )
            nestedScrollView.post { nestedScrollView.scrollTo(0, scrollY) }
        })
        viewModel.wifiSpeeds.observe(this, Observer { speeds ->
            if (!areWifiSpeedsEqual(lastWifiSpeeds, speeds)) {
                val scrollY = nestedScrollView.scrollY
                val gridData = mutableListOf<WifiSpeedCell>()
                var maxSpeed = -100.0
                var maxX = 0
                var maxY = 0
                for (i in speeds.indices) {
                    for (j in speeds[i].indices) {
                        val speed = speeds[i][j]
                        gridData.add(WifiSpeedCell(i, j, speed))
                        if (speed > maxSpeed) {
                            maxSpeed = speed
                            maxX = j
                            maxY = i
                        }
                    }
                }
                wifiSpeedAdapter.setMaxSpeedPosition(maxX, maxY)
                wifiSpeedAdapter.submitList(gridData)
                lastWifiSpeeds = speeds.map { it.copyOf() }.toTypedArray()
                nestedScrollView.post { nestedScrollView.scrollTo(0, scrollY) }
            }
        })

        // Xử lý Switch
        realtimeSwitch.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked) {
                viewModel.startPolling()
            } else {
                viewModel.stopPolling()
            }
        }

        // Xử lý sự kiện nhấn nút Stop
        stopButton.setOnClickListener {
            sendStopCommand()
        }

        // Bắt đầu polling
        viewModel.startPolling()
    }

    private fun sendStopCommand() {
        val queue = Volley.newRequestQueue(this)
        val url = "http://$serverIp/command"
        val request = object : StringRequest(
            Method.POST, url,
            { response ->
                statusText.text = "Status: Stopped"
                statusText.setTextColor(getColor(android.R.color.holo_green_dark))
                stopButton.isEnabled = false
            },
            { error ->
                statusText.text = "Error stopping robot: ${error.message}"
                statusText.setTextColor(getColor(android.R.color.holo_red_dark))
            }
        ) {
            override fun getParams(): Map<String, String> {
                return mapOf("action" to "stop")
            }
        }
        queue.add(request)
    }

    private fun updateUI(data: RobotData) {
        val scrollY = nestedScrollView.scrollY
        positionText.text = "Position: (${data.robotX}, ${data.robotY})"
        headingText.text = "Heading: ${data.robotHeading}°"
        wifiSpeedText.text = "WiFi RSSI: ${"%.1f".format(data.wifiSpeed)} dBm"
        maxWifiSpeedText.text = "Max WiFi RSSI: ${"%.1f".format(data.maxWifiSpeed)} dBm at (${data.maxWifiSpeedX}, ${data.maxWifiSpeedY})"
        distanceText.text = "Distances: L:${"%.1f".format(data.distanceLeft)} R:${"%.1f".format(data.distanceRight)} F:${"%.1f".format(data.distanceFront)}"
        directionText.text = "Direction: ${data.direction}"
        statusText.text = if (data.isFinished) "Status: Finished" else "Status: Running"
        statusText.setTextColor(if (data.isFinished) getColor(android.R.color.holo_green_dark) else getColor(android.R.color.holo_blue_dark))
        mapView.updateMap(data.grid, data.robotX, data.robotY, data.robotHeading, data.isFinished)
        nestedScrollView.post { nestedScrollView.scrollTo(0, scrollY) }
    }

    override fun onDestroy() {
        super.onDestroy()
        viewModel.stopPolling()
    }

    private fun areWifiSpeedsEqual(old: Array<DoubleArray>?, new: Array<DoubleArray>): Boolean {
        if (old == null) return false
        if (old.size != new.size) return false
        for (i in old.indices) {
            if (old[i].size != new[i].size) return false
            for (j in old[i].indices) {
                if (old[i][j] != new[i][j]) return false
            }
        }
        return true
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
