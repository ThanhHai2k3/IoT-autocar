package com.example.robotmonitor

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import com.android.volley.Request
import com.android.volley.toolbox.JsonObjectRequest
import com.android.volley.toolbox.Volley
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlin.collections.fill
import kotlin.collections.indices
import kotlin.ranges.until

class RobotViewModel(application: Application) : AndroidViewModel(application) {
    private val _robotData = MutableLiveData<RobotData>()
    val robotData: LiveData<RobotData> get() = _robotData

    private val _isLoading = MutableLiveData<Boolean>()
    val isLoading: LiveData<Boolean> get() = _isLoading

    private val _error = MutableLiveData<String?>()
    val error: LiveData<String?> get() = _error

    private val wifiSpeedMap = Array(10) { DoubleArray(10) { -100.0 } }
    private val _wifiSpeeds = MutableLiveData<Array<DoubleArray>>()
    val wifiSpeeds: LiveData<Array<DoubleArray>> get() = _wifiSpeeds

    private var pollingJob: Job? = null
    private val serverUrl = "http://172.20.10.2/data" // Cần xác nhận IP ESP32

    data class RobotData(
        val robotX: Int = 0,
        val robotY: Int = 0,
        val robotHeading: Int = 0,
        val isFinished: Boolean = false,
        val wifiSpeed: Double = -100.0,
        val maxWifiSpeed: Double = -100.0,
        val maxWifiSpeedX: Int = 0,
        val maxWifiSpeedY: Int = 0,
        val distanceLeft: Double = 0.0,
        val distanceRight: Double = 0.0,
        val distanceFront: Double = 0.0,
        val direction: String = "",
        val grid: Array<IntArray> = Array(10) { IntArray(10) }
    )

    fun startPolling() {
        stopPolling()
        pollingJob = CoroutineScope(Dispatchers.Main).launch {
            while (isActive) {
                fetchData()
                delay(1000)
            }
        }
    }

    fun stopPolling() {
        pollingJob?.cancel()
        pollingJob = null
    }

    private fun fetchData() {
        _isLoading.value = true
        val queue = Volley.newRequestQueue(getApplication())
        val jsonRequest = JsonObjectRequest(
            Request.Method.GET, serverUrl, null,
            { response ->
                try {
                    val grid = Array(10) { IntArray(10) }
                    val gridArray = response.getJSONArray("grid")
                    for (i in 0 until 10) {
                        val row = gridArray.getJSONArray(i)
                        for (j in 0 until 10) {
                            grid[i][j] = row.getInt(j)
                        }
                    }

                    val robotX = response.getInt("robotX")
                    val robotY = response.getInt("robotY")
                    val wifiSpeed = response.getDouble("wifiSpeed")
                    val resetFlag = response.getBoolean("resetFlag") // Lấy resetFlag từ JSON

                    // Chỉ reset wifiSpeedMap nếu resetFlag là true
                    if (resetFlag) {
                        for (i in wifiSpeedMap.indices) {
                            wifiSpeedMap[i].fill(-100.0)
                        }
                        _wifiSpeeds.value = wifiSpeedMap
                    }

                    // Cập nhật wifiSpeedMap với giá trị mới
                    wifiSpeedMap[robotY][robotX] = wifiSpeed

                    var maxWifiSpeed = -100.0
                    var maxWifiSpeedX = 0
                    var maxWifiSpeedY = 0
                    for (i in 0 until 10) {
                        for (j in 0 until 10) {
                            if (wifiSpeedMap[i][j] > maxWifiSpeed) {
                                maxWifiSpeed = wifiSpeedMap[i][j]
                                maxWifiSpeedX = j
                                maxWifiSpeedY = i
                            }
                        }
                    }

                    _wifiSpeeds.value = wifiSpeedMap

                    val newData = RobotData(
                        robotX = robotX,
                        robotY = robotY,
                        robotHeading = response.getInt("robotHeading"),
                        isFinished = response.getBoolean("isFinished"),
                        wifiSpeed = wifiSpeed,
                        maxWifiSpeed = maxWifiSpeed,
                        maxWifiSpeedX = maxWifiSpeedX,
                        maxWifiSpeedY = maxWifiSpeedY,
                        distanceLeft = response.getDouble("distanceLeft"),
                        distanceRight = response.getDouble("distanceRight"),
                        distanceFront = response.getDouble("distanceFront"),
                        direction = response.getString("direction"),
                        grid = grid
                    )

                    _robotData.value = newData
                    _error.value = null
                } catch (e: Exception) {
                    _error.value = "Error parsing JSON: ${e.message}"
                }
                _isLoading.value = false
            },
            { error ->
                _error.value = "Network error: ${error.message}"
                _isLoading.value = false
            }
        )
        queue.add(jsonRequest)
    }
}