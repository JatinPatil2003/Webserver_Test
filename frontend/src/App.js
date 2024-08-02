// src/App.js
import React from 'react';
import './App.css';
import MapView from './MapView';

function App() {
    return (
        <div className="App">
            <header className="App-header">
                <h1>Robot Control Interface</h1>
                <MapView />
            </header>
        </div>
    );
}

export default App;
