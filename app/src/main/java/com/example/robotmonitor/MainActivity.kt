package com.example.robotmonitor

import android.os.Bundle
import android.widget.Button
import android.widget.ProgressBar
import android.widget.TextView
import androidx.activity.enableEdgeToEdge
import androidx.activity.viewModels
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.widget.NestedScrollView
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
}
