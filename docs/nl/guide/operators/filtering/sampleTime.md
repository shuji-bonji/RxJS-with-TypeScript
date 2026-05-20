---
description: "De sampleTime operator is een RxJS filteroperator die periodiek de laatste waarden van de stream samplet met gespecificeerde tijdsintervallen. Het is ideaal voor het maken van periodieke momentopnames."
---

# sampleTime - periodiek de laatste waarde krijgen

De `sampleTime` operator neemt periodiek **een monster** van de laatste waarde van de bron Observable met **gespecificeerde tijdsintervallen** en voert dit uit.
Net als een periodieke momentopname haalt het de meest recente waarde op dat moment op.

## 🔰 Basissyntaxis en gebruik

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Monsters van seconde tot seconde');
});
```

**Werking**:.
1. timer gaat periodiek elke 2 seconden af
2. uitvoer als er op dat moment een recente klikgebeurtenis is
3. als er geen waarde is tijdens de sample-periode, geen uitvoer

> In het bovenstaande voorbeeld is het afmelden van `fromEvent` weggelaten voor de eenvoud van de uitleg. Gebruik in echte code `takeUntil(destroy$)`, `take(N)` of `Subscription.unsubscribe()` om de levenscyclus expliciet te beheren. Meer informatie: [Moeilijkheden overwinnen: levenscyclusbeheer](/nl/guide/overcoming-difficulties/lifecycle-management.md)

[🌐 Officiële RxJS documentatie - `sampleTime`](https://rxjs.dev/api/operators/sampleTime)

## 💡 Typische gebruikspatronen

- **Recurrent sensor data acquisition**: elke seconde up-to-date temperatuur- en locatie-informatie.
- Realtime dashboard**: regelmatige statusupdates
- **Prestatiemonitoring**: metriek verzamelen met regelmatige tussenpozen
- **Game frame verwerking**: periodieke bemonstering voor FPS-controle

## Praktisch codevoorbeeld 1: Periodieke bemonstering van muispositie

Dit is een voorbeeld van het elke seconde samplen van de muispositie.


```ts
import { fromEvent } from 'rxjs';
import { sampleTime, map } from 'rxjs';

// UIAanmaak
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Muispositie bemonsteren (1(elke seconde)';
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
area.textContent = 'Muis bewegen binnen dit gebied';
container.appendChild(area);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.maxHeight = '150px';
output.style.overflow = 'auto';
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

let sampleCount = 0;

// Gebeurtenis muisbeweging
fromEvent<MouseEvent>(area, 'mousemove').pipe(
  map(event => ({
    x: event.offsetX,
    y: event.offsetY,
    timestamp: Date.now()
  })),
  sampleTime(1000) // 1Elke seconde bemonstering
).subscribe(pos => {
  sampleCount++;
  const log = document.createElement('div');
  log.style.padding = '5px';
  log.style.borderBottom = '1px solid #eee';
  log.innerHTML = `
    <strong>Monsters #${sampleCount}</strong>
    [${new Date(pos.timestamp).toLocaleTimeString()}]
    Positie: (${pos.x}, ${pos.y})
  `;
  output.insertBefore(log, output.firstChild);

  // Max.10Weergave van maximaal
  while (output.children.length > 10) {
    output.removeChild(output.lastChild!);
  }
});
```

- Als de muis voortdurend wordt bewogen, wordt alleen de huidige laatste positie elke seconde gesampled.
- Als de muis een seconde lang niet wordt bewogen, wordt er voor die periode niets uitgevoerd.

## Praktisch codevoorbeeld 2: dashboard met real-time gegevens

Dit voorbeeld laat zien hoe sensorgegevens periodiek kunnen worden bemonsterd en weergegeven op een dashboard.

```ts
import { interval } from 'rxjs';
import { sampleTime, map } from 'rxjs';

// UIAanmaak
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Sensor bewakingsdashboard';
container.appendChild(title);

const dashboard = document.createElement('div');
dashboard.style.display = 'grid';
dashboard.style.gridTemplateColumns = '1fr 1fr';
dashboard.style.gap = '10px';
dashboard.style.marginTop = '10px';
container.appendChild(dashboard);

// Dashboardkaart maken
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

const tempValue = createCard('Temperatuur', '°C');
const humidityValue = createCard('Vochtigheid', '%');
const pressureValue = createCard('Barometrische druk', 'hPa');
const lightValue = createCard('Verlichtingssterkte', 'lux');

// Gegevensstroom sensor (100msElke seconde bijgewerkt)
const sensorData$ = interval(100).pipe(
  map(() => ({
    temperature: (20 + Math.random() * 10).toFixed(1),
    humidity: (40 + Math.random() * 40).toFixed(1),
    pressure: (1000 + Math.random() * 30).toFixed(1),
    light: Math.floor(Math.random() * 1000)
  }))
);

// 2Bemonsterd en bijgewerkt dashboard elke seconde
sensorData$.pipe(
  sampleTime(2000)
).subscribe(data => {
  tempValue.textContent = data.temperature;
  humidityValue.textContent = data.humidity;
  pressureValue.textContent = data.pressure;
  lightValue.textContent = data.light.toString();

  // Animatie-effect
  [tempValue, humidityValue, pressureValue, lightValue].forEach(elem => {
    elem.style.color = '#2196F3';
    setTimeout(() => {
      elem.style.color = 'black';
    }, 500);
  });
});
```

- De sensorgegevens worden elke 100 ms bijgewerkt, terwijl het dashboard elke 2 seconden wordt voorzien van bemonsterde waarden.
- De prestaties kunnen worden geoptimaliseerd door hoogfrequente gegevensstromen met de juiste intervallen weer te geven.

## Vergelijking met vergelijkbare operatoren

### sampleTime vs throttleTime vs auditTime

```ts
import { interval } from 'rxjs';
import { sampleTime, throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, ...

// sampleTime: 1Elke seconde bemonstering van de laatste waarde op dat moment
source$.pipe(
  sampleTime(1000)
).subscribe(val => console.log('sampleTime:', val));
// Uitvoer voorbeelden: 2, 5, 8(1Momentopname elke seconde)

// throttleTime: Nadat de eerste waarde is uitgevoerd,1Genegeerd gedurende 2 seconden nadat de eerste waarde is uitgevoerd
source$.pipe(
  throttleTime(1000)
).subscribe(val => console.log('throttleTime:', val));
// Uitvoer voorbeelden: 0, 3, 6, 9(eerste waarde van elke periode)

// auditTime: Uitvoer van de laatste waarde van de periode1seconden na de eerste waarde wordt de laatste waarde van de periode uitgevoerd
source$.pipe(
  auditTime(1000)
).subscribe(val => console.log('auditTime:', val));
// Uitvoer voorbeelden: 2, 5, 8(laatste waarde van elke periode)
```

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Monsters van seconde tot seconde');
});
```

**visuele verschillen**:.

Ingang: --|1|2|3|---|4|5|6|---|7|8|9|
      0s  1s      2s      3s

sampleTime(1s):  -------|3|-------|6|-------|9|
                 (Periodieke bemonstering)

throttleTime(1s): |1|--------------|4|--------------|7|
                  (Genegeerd tijdens de periode aan het begin)

auditTime(1s):    -------|3|-------|6|-------|9|
                  (Laatste waarde aan het einde van de periode)
```

## ⚠️ Opmerkingen.

### 1. geen waarde tijdens de steekproefperiode

Als er geen nieuwe waarden zijn tijdens de bemonsteringstijd, wordt er geen uitvoer geproduceerd.

```

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('Monsters genomen');
});
// 2Gedurende seconden1Geen uitvoer als er niet wordt geklikt
```

### 2. Wacht op de eerste bemonsteringstijd

De `sampleTime` voert niets uit totdat de opgegeven tijd is verstreken.

```ts
import { interval } from 'rxjs';
import { sampleTime } from 'rxjs';

interval(100).pipe(
  sampleTime(1000)
).subscribe(console.log);
// Eerste waarde is1seconden nadat de eerste waarde is uitgevoerd
```

### 3. completionTime

Als een bron voltooid is, wordt de voltooiing niet doorgegeven tot de volgende monstertiming.

```ts
import { of } from 'rxjs';
import { sampleTime, delay } from 'rxjs';

of(1, 2, 3).pipe(
  delay(100),
  sampleTime(1000)
).subscribe({
  next: console.log,
  complete: () => console.log('Voltooid')
});
// 1Seconden later: 3
// 1Seconden later: Voltooid
```

### 4. geheugengebruik

De geheugenefficiëntie is goed omdat intern slechts één laatste waarde wordt bijgehouden.

```ts
import { interval } from 'rxjs';
import { sampleTime } from 'rxjs';

// Hoogfrequente stroom (10msper seconde)
interval(10).pipe(
  sampleTime(1000) // 1Elke seconde bemonstering
).subscribe(console.log);
// Geheugen bewaart alleen de meest recente1Alleen de twee meest recente waarden worden in het geheugen bewaard
```

## Verschillen met sample

`sample` gebruikt een andere Observable als trigger, terwijl `sampleTime` een vast tijdsinterval gebruikt.

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Monsters van seconde tot seconde');
});
```

TABEL 12

## 📚 Gerelateerde operatoren.

- **[sample](https://rxjs.dev/api/operators/sample)** - Een andere Observable samplen als trigger (officiële documentatie).
- **[throttleTime](. /throttleTime)** - Verkrijg de eerste waarde aan het begin van de periode.
- **[auditTime](. /auditTime)** - Verkrijg de laatste waarde aan het einde van de periode.
- **[debounceTime](. /debounceTime)** - waarde uitgeven na stilstand

## Samenvatting.

De `sampleTime` operator bemonstert periodiek de laatste waarde in het opgegeven tijdsinterval.

- Ideaal voor het maken van periodieke momentopnamen.
- ✅ Nuttig voor het uitdunnen van hoogfrequente stromen
- ✅ Geheugenefficiënt (er wordt slechts één laatste waarde bewaard)
- Ideaal voor dashboards en monitoring
- ⚠️ Als er geen waarden beschikbaar zijn tijdens de sample-periode, wordt er niets uitgevoerd
- ⚠️ Er is een wachtperiode tot de eerste steekproef
- ⚠️ De voltooiing wordt doorgegeven bij de volgende sample-timing
