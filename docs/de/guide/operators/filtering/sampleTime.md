---
description: "Der sampleTime-Operator ist ein RxJS-Filteroperator, der in regelmäßigen Abständen die neuesten Werte des Streams in bestimmten Zeitintervallen abfragt. Er eignet sich ideal für die Erstellung regelmäßiger Schnappschüsse."
---

# sampleTime - holt periodisch den letzten Wert

Der "sampleTime"-Operator **ermittelt** periodisch in **bestimmten Zeitabständen** den neuesten Wert des Observable und gibt ihn aus.
Wie bei einem periodischen Schnappschuss wird der aktuellste Wert zu diesem Zeitpunkt abgerufen.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Stichproben im Sekundentakt');
});
```

**Ablauf der Operation**:.
1. Timer feuert periodisch alle 2 Sekunden
2. Ausgabe, wenn es zu diesem Zeitpunkt ein aktuelles Klick-Ereignis gibt
3. wenn während der Probezeit kein Wert vorhanden ist, keine Ausgabe

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Stichproben im Sekundentakt');
});
```ts
import { interval } from 'rxjs';
import { sampleTime, map } from 'rxjs';

// UIErstellen
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'センサーモニタリングダッシュボード';
container.appendChild(title);

const dashboard = document.createElement('div');
dashboard.style.display = 'grid';
dashboard.style.gridTemplateColumns = '1fr 1fr';
dashboard.style.gap = '10px';
dashboard.style.marginTop = '10px';
container.appendChild(dashboard);

// ダッシュボードカードErstellen
function createCard(label: string, unit: string) {
  const card = document.createElement('div');
  card.style.padding = '20px';
  card.style.border = '2px solid #2196F3';
  card.style.borderRadius = '8px';
  card.style.backgroundColor = '#E3F2FD';

  const labelDiv = document.createElement('div');
  labelDiv.textContent = label;
  labelDiv.style.fontSize = '14px';
  labelDiv.style.color = '#666';
  card.appendChild(labelDiv);

  const valueDiv = document.createElement('div');
  valueDiv.style.fontSize = '32px';
  valueDiv.style.fontWeight = 'bold';
  valueDiv.style.marginTop = '10px';
  valueDiv.textContent = '--';
  card.appendChild(valueDiv);

  const unitDiv = document.createElement('div');
  unitDiv.textContent = unit;
  unitDiv.style.fontSize = '14px';
  unitDiv.style.color = '#666';
  card.appendChild(unitDiv);

  dashboard.appendChild(card);
  return valueDiv;
}

const tempValue = createCard('温度', '°C');
const humidityValue = createCard('湿度', '%');
const pressureValue = createCard('気圧', 'hPa');
const lightValue = createCard('照度', 'lux');

// センサーデータストリーム（100msごとに更新）
const sensorData$ = interval(100).pipe(
  map(() => ({
    temperature: (20 + Math.random() * 10).toFixed(1),
    humidity: (40 + Math.random() * 40).toFixed(1),
    pressure: (1000 + Math.random() * 30).toFixed(1),
    light: Math.floor(Math.random() * 1000)
  }))
);

// 2秒ごとにサンプリングしてダッシュボードを更新
sensorData$.pipe(
  sampleTime(2000)
).subscribe(data => {
  tempValue.textContent = data.temperature;
  humidityValue.textContent = data.humidity;
  pressureValue.textContent = data.pressure;
  lightValue.textContent = data.light.toString();

  // アニメーション効果
  [tempValue, humidityValue, pressureValue, lightValue].forEach(elem => {
    elem.style.color = '#2196F3';
    setTimeout(() => {
      elem.style.color = 'black';
    }, 500);
  });
});
```

> Das obige Beispiel lässt der Einfachheit halber das Abbestellen von `fromEvent` weg. In echtem Code sollten Sie `takeUntil(destroy$)`, `take(N)` oder `Subscription.unsubscribe()` verwenden, um den Lebenszyklus explizit zu verwalten. Weitere Informationen: [Schwierigkeiten überwinden: Lebenszyklus-Management](/de/guide/overcoming-difficulties/lifecycle-management.md)

[🌐 Offizielle RxJS Dokumentation - `sampleTime`](https://rxjs.dev/api/operators/sampleTime)

## 💡 Typische Nutzungsmuster

- **Wiederkehrende Sensordatenerfassung**: aktuelle Temperatur- und Standortinformationen im Sekundentakt.
- **Echtzeit-Dashboard**: regelmäßige Status-Updates
- **Leistungsüberwachung**: Sammlung von Metriken in regelmäßigen Abständen
- Verarbeitung von Spielframes**: periodische Abtastung zur FPS-Kontrolle

## 🧠 Praktisches Codebeispiel 1: Periodische Abtastung der Mausposition

Dieses Beispiel zeigt, wie die Mausposition jede Sekunde abgetastet wird.

```ts
import { fromEvent } from 'rxjs';
import { sampleTime, map } from 'rxjs';

// UIErstellung
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Abtasten der Mausposition (1(jede Sekunde)';
container.appendChild(title);

const area = document.createElement('div');
area.style.width = '100%';
area.style.height = '300px';
area.style.border = '2px solid #4CAF50';
area.style.backgroundColor = '#f5f5f5';
area.style.display = 'flex';
area.style.alignItems = 'center';
area.style.justifyContent = 'center';
area.style.fontSize = '18px';
area.textContent = 'Bewegen Sie die Maus in diesem Bereich';
container.appendChild(area);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.maxHeight = '150px';
output.style.overflow = 'auto';
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

let sampleCount = 0;

// Mausbewegungs-Ereignis
fromEvent<MouseEvent>(area, 'mousemove').pipe(
  map(event => ({
    x: event.offsetX,
    y: event.offsetY,
    timestamp: Date.now()
  })),
  sampleTime(1000) // 1Abtastung jede Sekunde
).subscribe(pos => {
  sampleCount++;
  const log = document.createElement('div');
  log.style.padding = '5px';
  log.style.borderBottom = '1px solid #eee';
  log.innerHTML = `
    <strong>Abtastung #${sampleCount}</strong>
    [${new Date(pos.timestamp).toLocaleTimeString()}]
    Standort: (${pos.x}, ${pos.y})
  `;
  output.insertBefore(log, output.firstChild);

  // Max.10Anzeige von bis zu
  while (output.children.length > 10) {
    output.removeChild(output.lastChild!);
  }
});
```

- Wenn die Maus kontinuierlich bewegt wird, wird nur die aktuellste Position jede Sekunde abgetastet.
- Wenn die Maus eine Sekunde lang nicht bewegt wird, wird für diesen Zeitraum nichts ausgegeben.

## 🎯 Praktisches Code-Beispiel 2: Echtzeit-Daten-Dashboard

Dieses Beispiel zeigt, wie Sensordaten periodisch abgetastet und in einem Dashboard angezeigt werden können.

```ts
import { interval } from 'rxjs';
import { sampleTime, map } from 'rxjs';

// UIErstellung
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Dashboard für die Sensorüberwachung';
container.appendChild(title);

const dashboard = document.createElement('div');
dashboard.style.display = 'grid';
dashboard.style.gridTemplateColumns = '1fr 1fr';
dashboard.style.gap = '10px';
dashboard.style.marginTop = '10px';
container.appendChild(dashboard);

// Erstellung von Dashboard-Karten
function createCard(label: string, unit: string) {
  const card = document.createElement('div');
  card.style.padding = '20px';
  card.style.border = '2px solid #2196F3';
  card.style.borderRadius = '8px';
  card.style.backgroundColor = '#E3F2FD';

  const labelDiv = document.createElement('div');
  labelDiv.textContent = label;
  labelDiv.style.fontSize = '14px';
  labelDiv.style.color = '#666';
  card.appendChild(labelDiv);

  const valueDiv = document.createElement('div');
  valueDiv.style.fontSize = '32px';
  valueDiv.style.fontWeight = 'bold';
  valueDiv.style.marginTop = '10px';
  valueDiv.textContent = '--';
  card.appendChild(valueDiv);

  const unitDiv = document.createElement('div');
  unitDiv.textContent = unit;
  unitDiv.style.fontSize = '14px';
  unitDiv.style.color = '#666';
  card.appendChild(unitDiv);

  dashboard.appendChild(card);
  return valueDiv;
}

const tempValue = createCard('Temperatur', '°C');
const humidityValue = createCard('Luftfeuchtigkeit', '%');
const pressureValue = createCard('Barometrischer Druck', 'hPa');
const lightValue = createCard('Beleuchtungsstärke', 'lux');

// Sensordatenstrom (100msJede Aktualisierung)
const sensorData$ = interval(100).pipe(
  map(() => ({
    temperature: (20 + Math.random() * 10).toFixed(1),
    humidity: (40 + Math.random() * 40).toFixed(1),
    pressure: (1000 + Math.random() * 30).toFixed(1),
    light: Math.floor(Math.random() * 1000)
  }))
);

// 2Abtastung und Aktualisierung des Dashboards jede Sekunde
sensorData$.pipe(
  sampleTime(2000)
).subscribe(data => {
  tempValue.textContent = data.temperature;
  humidityValue.textContent = data.humidity;
  pressureValue.textContent = data.pressure;
  lightValue.textContent = data.light.toString();

  // Animationseffekt
  [tempValue, humidityValue, pressureValue, lightValue].forEach(elem => {
    elem.style.color = '#2196F3';
    setTimeout(() => {
      elem.style.color = 'black';
    }, 500);
  });
});
```

- Die Sensordaten werden alle 100 ms aktualisiert, während das Dashboard alle 2 Sekunden mit den abgetasteten Werten aktualisiert wird.
- Die Leistung kann optimiert werden, indem hochfrequente Datenströme in angemessenen Abständen angezeigt werden.

## 🆚 Vergleich mit ähnlichen Betreibern

### sampleTime vs throttleTime vs auditTime

```ts
import { interval } from 'rxjs';
import { sampleTime, throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, ...

// sampleTime: 1Abtastung des letzten Wertes zu diesem Zeitpunkt jede Sekunde
source$.pipe(
  sampleTime(1000)
).subscribe(val => console.log('sampleTime:', val));
// Beispiele für die Ausgabe: 2, 5, 8(1Schnappschuss jede Sekunde)

// throttleTime: Nachdem der erste Wert ausgegeben wurde,1Ignoriert für 2 Sekunden nach der Ausgabe des ersten Wertes
source$.pipe(
  throttleTime(1000)
).subscribe(val => console.log('throttleTime:', val));
// Beispiele für die Ausgabe: 0, 3, 6, 9(erster Wert jeder Periode)

// auditTime: Ausgabe des letzten Wertes der Periode1Sekunden nach dem ersten Wert wird der letzte Wert der Periode ausgegeben
source$.pipe(
  auditTime(1000)
).subscribe(val => console.log('auditTime:', val));
// Beispiele für die Ausgabe: 2, 5, 8(letzter Wert jeder Periode)
```

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Stichproben im Sekundentakt');
});
```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2秒ごとのサンプル');
});
```

**visuelle Unterschiede**:.

```
Eingabe: --|1|2|3|---|4|5|6|---|7|8|9|
      0s  1s      2s      3s

sampleTime(1s):  -------|3|-------|6|-------|9|
                 (Periodische Abtastung)

throttleTime(1s): |1|--------------|4|--------------|7|
                  (Ignoriert während der Periode durch den Anfang)

auditTime(1s):    -------|3|-------|6|-------|9|
                  (Letzter Wert am Ende der Periode)
```

## ⚠️ Anmerkungen.

### 1. kein Wert während des Stichprobenzeitraums

Wenn zum Zeitpunkt der Abtastung keine neuen Werte vorhanden sind, wird keine Ausgabe erzeugt.

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('Entnommene Stichproben');
});
// 2Während Sekunden1Keine Ausgabe, wenn keine Klicks gemacht werden
```

### 2. auf den ersten Abtastzeitpunkt warten

Die `sampleTime` gibt nichts aus, bis die angegebene Zeit verstrichen ist.

```ts
import { interval } from 'rxjs';
import { sampleTime } from 'rxjs';

interval(100).pipe(
  sampleTime(1000)
).subscribe(console.log);
// Erster Wert ist1Sekunden nach Ausgabe des ersten Wertes
```

### 3. completionTime

Wenn eine Quelle abgeschlossen ist, wird der Abschluss nicht bis zur nächsten Abtastzeit weitergegeben.

```ts
import { of } from 'rxjs';
import { sampleTime, delay } from 'rxjs';

of(1, 2, 3).pipe(
  delay(100),
  sampleTime(1000)
).subscribe({
  next: console.log,
  complete: () => console.log('Abgeschlossen')
});
// 1Sekunden später: 3
// 1Sekunden später: Abgeschlossen
```

### 4. Speichernutzung

Die Speichereffizienz ist gut, da intern nur ein letzter Wert gespeichert wird.

```ts
import { interval } from 'rxjs';
import { sampleTime } from 'rxjs';

// Hochfrequenzstrom (10mspro Sekunde)
interval(10).pipe(
  sampleTime(1000) // 1Abtastung jede Sekunde
).subscribe(console.log);
// Im Speicher wird nur der letzte Wert gespeichert1Nur die beiden jüngsten Werte werden im Speicher gehalten
```

## 💡 Unterschiede zum sample

`sample` verwendet ein anderes Observable als Auslöser, während `sampleTime` ein festes Zeitintervall verwendet.

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Stichproben im Sekundentakt');
});
---
description: sampleTimeオペレーターは、指定した時間間隔で定期的にストリームの最新値をサンプリングするRxJSフィルタリングオペレーターです。定期的なスナップショット取得に最適です。
---


```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Stichproben im Sekundentakt');
});
```ts
import { fromEvent } from 'rxjs';
import { sampleTime, map } from 'rxjs';

// UIErstellen
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'マウス位置サンプリング（1秒ごと）';
container.appendChild(title);

const area = document.createElement('div');
area.style.width = '100%';
area.style.height = '300px';
area.style.border = '2px solid #4CAF50';
area.style.backgroundColor = '#f5f5f5';
area.style.display = 'flex';
area.style.alignItems = 'center';
area.style.justifyContent = 'center';
area.style.fontSize = '18px';
area.textContent = 'この領域内でマウスを動かしてください';
container.appendChild(area);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.maxHeight = '150px';
output.style.overflow = 'auto';
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

let sampleCount = 0;

// マウス移動イベント
fromEvent<MouseEvent>(area, 'mousemove').pipe(
  map(event => ({
    x: event.offsetX,
    y: event.offsetY,
    timestamp: Date.now()
  })),
  sampleTime(1000) // 1秒ごとにサンプリング
).subscribe(pos => {
  sampleCount++;
  const log = document.createElement('div');
  log.style.padding = '5px';
  log.style.borderBottom = '1px solid #eee';
  log.innerHTML = `
    <strong>サンプル #${sampleCount}</strong>
    [${new Date(pos.timestamp).toLocaleTimeString()}]
    位置: (${pos.x}, ${pos.y})
  `;
  output.insertBefore(log, output.firstChild);

  // Max.10件まで表示
  while (output.children.length > 10) {
    output.removeChild(output.lastChild!);
  }
});
```

## 📚 Verwandte Operatoren.

- **[sample](https://rxjs.dev/api/operators/sample)** - Sampling eines anderen Observable als Auslöser (offizielle Dokumentation).
- **[throttleTime](. /throttleTime)** - Ermittelt den ersten Wert zu Beginn der Periode.
- **[auditTime](. /auditTime)** - Ermittelt den letzten Wert am Ende des Zeitraums
- **[debounceTime](. /debounceTime)** - gibt den Wert nach der Ruhezeit aus

## Zusammenfassung.

Der "sampleTime"-Operator tastet periodisch den letzten Wert im angegebenen Zeitintervall ab.

- ✅ Ideal, um periodische Schnappschüsse zu machen
- ✅ Nützlich zum Ausdünnen von Hochfrequenzströmen
- ✅ Speichereffizient (nur ein letzter Wert wird gespeichert)
- ✅ Ideal für Dashboards und Überwachung
- ⚠️ Wenn während des Stichprobenzeitraums keine Werte verfügbar sind, wird nichts ausgegeben.
- ⚠️ Es gibt eine Wartezeit bis zur ersten Stichprobe
- ⚠️ Die Fertigstellung wird zum nächsten Abtastzeitpunkt weitergegeben
