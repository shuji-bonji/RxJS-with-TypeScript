---
description: De audit operator is een RxJS filteroperator die alleen de laatste waarde uitgeeft binnen een periode die wordt gecontroleerd door een aangepaste Observable. Het is ideaal voor dynamische timingcontrole.
titleTemplate: ':title'
---

# audit - Geef de laatste waarde uit tijdens een periode gecontroleerd door een aangepaste Observable

De `audit` operator wacht tot een aangepaste Observable een waarde uitgeeft en geeft de **laatste waarde** van de bron tijdens die periode uit.
Terwijl `auditTime` met een vaste tijd controleert, kan `audit` **de periode dynamisch controleren met een Observable**.

## 🔰 Basissyntax en gebruik

```ts
import { fromEvent, interval } from 'rxjs';
import { audit } from 'rxjs';

// Klikgebeurtenis
const clicks$ = fromEvent(document, 'click');

// Scheid periode elke seconde
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('Klik is geregistreerd');
});
```

- Wanneer een klik optreedt, begint een periode van 1 seconde.
- Alleen de laatste klik tijdens die 1 seconde wordt uitgegeven.
- De volgende periode begint na 1 seconde.

[🌐 RxJS Officiële Documentatie - `audit`](https://rxjs.dev/api/operators/audit)

## 💡 Typische gebruikspatronen

- **Bemonsteren op dynamische intervallen**: Pas periode aan volgens belasting
- **Aangepaste timingcontrole**: Periodecontrole gebaseerd op andere Observables
- **Adaptieve gebeurtenisbeperking**: Uitdunnen volgens omstandigheden

## 🔍 Verschil met auditTime

| Operator | Periodecontrole | Gebruiksscenario |
|:---|:---|:---|
| `auditTime` | Vaste tijd (milliseconden) | Eenvoudige tijdgebaseerde controle |
| `audit` | **Aangepaste Observable** | **Dynamische periodecontrole** |

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, auditTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// auditTime - Vaste 1 seconde
clicks$.pipe(
  auditTime(1000)
).subscribe(() => console.log('Vaste 1 seconde'));

// audit - Dynamische periode
let period = 1000;
clicks$.pipe(
  audit(() => {
    period = Math.random() * 2000; // Willekeurige periode 0-2 seconden
    return timer(period);
  })
).subscribe(() => console.log(`Dynamische periode: ${period}ms`));
```

## 🧠 Praktisch codevoorbeeld: Dynamische bemonstering volgens belasting

Voorbeeld van het aanpassen van bemonsteringsinterval volgens systeembelasting.

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map } from 'rxjs';

// Maak UI
const output = document.createElement('div');
output.innerHTML = '<h3>Dynamische Bemonstering</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Wijzig Belasting';
document.body.appendChild(button);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
output.appendChild(statusDiv);

const logDiv = document.createElement('div');
logDiv.style.marginTop = '10px';
logDiv.style.maxHeight = '200px';
logDiv.style.overflow = 'auto';
output.appendChild(logDiv);

// Belastingsniveau (0: laag, 1: gemiddeld, 2: hoog)
let loadLevel = 0;

fromEvent(button, 'click').subscribe(() => {
  loadLevel = (loadLevel + 1) % 3;
  const levels = ['Lage Belasting', 'Gemiddelde Belasting', 'Hoge Belasting'];
  statusDiv.textContent = `Huidige belasting: ${levels[loadLevel]}`;
});

// Muisbewegingsgebeurtenis
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  audit(() => {
    // Pas periode aan volgens belasting
    const periods = [2000, 1000, 500]; // Lage belasting → lange periode, hoge belasting → korte periode
    return timer(periods[loadLevel]);
  }),
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe(pos => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Muispositie: (${pos.x}, ${pos.y})`;
  logDiv.insertBefore(log, logDiv.firstChild);

  // Toon maximaal 10 items
  while (logDiv.children.length > 10) {
    logDiv.removeChild(logDiv.lastChild!);
  }
});
```

- Bij lage belasting, dun uit met intervallen van 2 seconden (energiebesparingsmodus)
- Bij hoge belasting, bemonsteren fijn met intervallen van 500ms
- Periode kan dynamisch worden aangepast volgens belasting


## ⚠️ Opmerkingen

### 1. Eerste waarde wordt niet onmiddellijk uitgegeven

`audit` wacht tot de periode eindigt na ontvangst van de eerste waarde.

```ts
import { interval, timer } from 'rxjs';
import { audit, take } from 'rxjs';

interval(100).pipe(
  audit(() => timer(1000)),
  take(3)
).subscribe(val => {
  console.log(val);
});
// Output:
// 9  (na 1 seconde, laatste waarde van 0-9)
// 19 (na 2 seconden, laatste waarde van 10-19)
// 29 (na 3 seconden, laatste waarde van 20-29)
```

### 2. Duur Observable moet elke keer opnieuw worden gegenereerd

De functie doorgegeven aan `audit` moet **elke keer een nieuwe Observable retourneren**.

```ts
// ❌ Slecht voorbeeld: Hergebruik dezelfde Observable-instantie
const duration$ = timer(1000);
source$.pipe(
  audit(() => duration$) // Werkt niet na 2e keer
).subscribe();

// ✅ Goed voorbeeld: Genereer elke keer nieuwe Observable
source$.pipe(
  audit(() => timer(1000))
).subscribe();
```

## 🆚 Vergelijking met vergelijkbare operators

| Operator | Uitgavetiming | Uitgegeven waarde | Gebruiksscenario |
|:---|:---|:---|:---|
| `audit` | Aan periode **einde** | **Laatste** waarde binnen periode | Laatste status binnen periode ophalen |
| `throttle` | Aan periode **begin** | **Eerste** waarde binnen periode | Eerste van opeenvolgende gebeurtenissen ophalen |
| `debounce` | **Na stilte** | Waarde net voor stilte | Wacht op invoervoltooiing |
| `sample` | **Wanneer andere Observable vuurt** | Laatste waarde op dat moment | Periodieke snapshots |


## 📚 Gerelateerde operators

- **[auditTime](/nl/guide/operators/filtering/auditTime)** - Controle met vaste tijd (vereenvoudigde versie van `audit`)
- **[throttle](/nl/guide/operators/filtering/throttleTime)** - Geef eerste waarde uit aan begin van periode
- **[debounce](/nl/guide/operators/filtering/debounceTime)** - Geef waarde uit na stilte
- **[sample](/nl/guide/operators/filtering/sampleTime)** - Bemonsteren op timing van andere Observable

## Samenvatting

De `audit` operator geeft de laatste waarde uit binnen een periode die dynamisch wordt gecontroleerd door een aangepaste Observable.

- ✅ Dynamische periodecontrole mogelijk
- ✅ Adaptieve bemonstering volgens belasting
- ✅ Controle gebaseerd op andere streams
- ⚠️ Moet elke keer nieuwe Observable genereren
- ⚠️ Let op geheugen bij frequente emissies
