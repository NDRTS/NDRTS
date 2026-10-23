const express = require('express');
const path = require('path');
const { exec } = require('child_process');

const app = express();
const PORT = 3000;

// Serve static files from Vite build output
const staticPath = path.join(__dirname, '../webapp/dashboard-ui/dist');
console.log(`Serving static files from: ${staticPath}`);
app.use(express.static(staticPath));

app.post('/api/start-car', (req, res) => {
    exec('bash /home/ndrts/workspaces/NDRTS-BFMC2024/start_car.sh', (error, stdout, stderr) => {
        if (error) {
            console.error(`❌ Error: ${error.message}`);
            return res.status(500).json({ success: false, message: error.message });
        }
        if (stderr) {
            console.warn(`⚠️ Stderr: ${stderr}`);
        }
        console.log(`✅ Car started:\n${stdout}`);
        res.json({ success: true, message: stdout });
    });
});

app.listen(PORT, () => {
    console.log(`✅ Website available at http://localhost:${PORT}`);
});
