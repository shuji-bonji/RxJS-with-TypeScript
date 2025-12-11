---
description: De sampleTime operator is een RxJS filteroperator die periodiek de laatste waarde uit een stream bemonstert op gespecificeerde tijdsintervallen. Het is ideaal voor het nemen van periodieke snapshots.
titleTemplate: ':title | RxJS'
---

# sampleTime - Bemonsteren van laatste waarde op gespecificeerde tijdsintervallen

De `sampleTime` operator **bemonstert periodiek** en geeft de **laatste waarde** uit van de bron Observable op **gespecificeerde tijdsintervallen**.
Net als periodieke snapshots, haalt het de laatste waarde op dat moment op.

## 🔰 Basissyntax en gebruik

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('Bemonsteren elke 2 seconden');
});
```

**Werkingsstroom**:
1. Timer vuurt periodiek elke 2 seconden
2. Als er op dat moment een laatste klikgebeurtenis is, geef deze uit
3. Als er geen waarde is tijdens de bemonsteringsperiode, wordt niets uitgegeven

[🌐 RxJS Officiële Documentatie - `sampleTime`](https://rxjs.dev/api/operators/sampleTime)

## 💡 Typische gebruikspatronen

- **Periodieke sensordataverzameling**: Laatste temperatuur- of positie-informatie elke seconde
- **Realtime dashboard**: Periodieke statusupdates
- **Prestatiemonitoring**: Metriekenverzameling op regelmatige intervallen
- **Game frameverwerking**: Periodieke bemonstering voor FPS-controle

## 🧠 Praktisch codevoorbeeld: Periodieke muispositiebemonstering

Voorbeeld van het bemonsteren en weergeven van muispositie elke seconde.

```ts
import { fromEvent } from 'rxjs';
import { sampleTime, map } from 'rxjs';

// Maak UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Muispositie Bemonstering (elke seconde)';
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
area.textContent = 'Beweeg uw muis binnen dit gebied';
container.appendChild(area);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.maxHeight = '150px';
output.style.overflow = 'auto';
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

let sampleCount = 0;

// Muisbewegingsgebeurtenis
fromEvent<MouseEvent>(area, 'mousemove').pipe(
  map(event => ({
    x: event.offsetX,
    y: event.offsetY,
    timestamp: Date.now()
  })),
  sampleTime(1000) // Bemonsteren elke seconde
).subscribe(pos => {
  sampleCount++;
  const log = document.createElement('div');
  log.style.padding = '5px';
  log.style.borderBottom = '1px solid #eee';
  log.innerHTML = `
    <strong>Sample #${sampleCount}</strong>
    [${new Date(pos.timestamp).toLocaleTimeString()}]
    Positie: (${pos.x}, ${pos.y})
  `;
  output.insertBefore(log, output.firstChild);

  // Toon maximaal 10 items
  while (output.children.length > 10) {
    output.removeChild(output.lastChild!);
  }
});
```

- Zelfs als u de muis blijft bewegen, wordt alleen de laatste positie op dat moment elke seconde bemonsterd.
- Als u de muis niet beweegt gedurende 1 seconde, wordt niets uitgegeven tijdens die periode.

## 🆚 Vergelijking met vergelijkbare operators

### sampleTime vs throttleTime vs auditTime

| Operator | Vuurtiming | Uitgegeven waarde | Gebruiksscenario |
|:---|:---|:---|:---|
| `sampleTime(1000)` | **Regelmatige timing elke 1 seconde** | Laatste waarde op dat moment | Periodieke snapshots |
| `throttleTime(1000)` | Negeer voor 1 seconde na waarde-ontvangst | Eerste waarde aan begin van periode | Gebeurtenisuitdunning |
| `auditTime(1000)` | 1 seconde na waarde-ontvangst | Laatste waarde binnen periode | Laatste status binnen periode |

**Visueel verschil**:

```
Invoer: --|1|2|3|---|4|5|6|---|7|8|9|
      0s  1s      2s      3s

sampleTime(1s):  -------|3|-------|6|-------|9|
                 (periodiek bemonsteren)

throttleTime(1s): |1|--------------|4|--------------|7|
                  (laat eerste door en negeer tijdens periode)

auditTime(1s):    -------|3|-------|6|-------|9|
                  (laatste waarde aan einde van periode)
```

## ⚠️ Opmerkingen

### 1. Wanneer er geen waarde is tijdens bemonsteringsperiode

Als er geen nieuwe waarde is op de bemonsteringstiming, wordt niets uitgegeven.

### 2. Wacht tot eerste bemonsteringstiming

`sampleTime` geeft niets uit tot de gespecificeerde tijd is verstreken.

### 3. Voltooiingstiming

Zelfs als de bron voltooit, wordt voltooiing niet doorgegeven tot de volgende bemonsteringstiming.

### 4. Geheugengebruik

Geheugenefficiëntie is goed omdat het intern slechts één laatste waarde vasthoudt.

## 💡 Verschil met sample

`sample` gebruikt een andere Observable als trigger, terwijl `sampleTime` vaste tijdsintervallen gebruikt.

```ts
import { interval, fromEvent } from 'rxjs';
import { sample, sampleTime } from 'rxjs';

const source$ = interval(100);

// sampleTime: Vast tijdsinterval (elke 1 seconde)
source$.pipe(
  sampleTime(1000)
).subscribe(val => console.log('sampleTime:', val));

// sample: Gebruik andere Observable als trigger
const clicks$ = fromEvent(document, 'click');
source$.pipe(
  sample(clicks$)
).subscribe(val => console.log('sample:', val));
// Geef laatste waarde op dat moment uit elke keer dat u klikt
```

| Operator | Trigger | Gebruiksscenario |
|:---|:---|:---|
| `sampleTime(ms)` | Vast tijdsinterval | Periodieke bemonstering |
| `sample(notifier$)` | Andere Observable | Bemonstering op dynamische timing |

## 📚 Gerelateerde operators

- **[sample](https://rxjs.dev/api/operators/sample)** - Bemonsteren met andere Observable als trigger (officiële documentatie)
- **[throttleTime](/nl/guide/operators/filtering/throttleTime)** - Haal eerste waarde aan begin van periode
- **[auditTime](/nl/guide/operators/filtering/auditTime)** - Haal laatste waarde aan einde van periode
- **[debounceTime](/nl/guide/operators/filtering/debounceTime)** - Geef waarde uit na stilte

## Samenvatting

De `sampleTime` operator bemonstert periodiek de laatste waarde op gespecificeerde tijdsintervallen.

- ✅ Ideaal voor periodieke snapshot-acquisitie
- ✅ Effectief voor uitdunnen van hoogfrequente streams
- ✅ Goede geheugenefficiëntie (houdt slechts 1 laatste waarde vast)
- ✅ Ideaal voor dashboards en monitoring
- ⚠️ Geeft niets uit als er geen waarde is tijdens bemonsteringsperiode
- ⚠️ Wachttijd tot eerste sample
- ⚠️ Voltooiing propageert op volgende bemonsteringstiming
