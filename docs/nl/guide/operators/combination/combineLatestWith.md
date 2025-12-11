---
description: combineLatestWith is een RxJS combinatie-operator die de originele Observable combineert met de laatste waarden van andere Observables om een nieuwe stream te vormen. Het is de Pipeable Operator versie van de Creation Function combineLatest, en is ideaal voor realtime formuliervalidatie, combineren van meerdere sensorgegevens, combineren van zoekfilters en andere situaties waarin u de laatste waarden van andere streams wilt integreren terwijl u de hoofdstream transformeert of verwerkt.
titleTemplate: ':title | RxJS'
---

# combineLatestWith - Combineer laatste waarden binnen een pipeline

De `combineLatestWith` operator combineert de **laatste waarden** van de originele Observable en andere gespecificeerde Observables tot een nieuwe stream.
Dit is de Pipeable Operator versie van de Creation Function `combineLatest`.

## 🔰 Basissyntax en gebruik

```ts
import { interval } from 'rxjs';
import { combineLatestWith, map } from 'rxjs';

const source1$ = interval(1000); // 0, 1, 2, ...
const source2$ = interval(1500); // 0, 1, 2, ...

source1$
  .pipe(
    combineLatestWith(source2$),
    map(([val1, val2]) => `Stream1: ${val1}, Stream2: ${val2}`)
  )
  .subscribe(console.log);

// Voorbeeldoutput:
// Stream1: 0, Stream2: 0
// Stream1: 1, Stream2: 0
// Stream1: 2, Stream2: 0
// Stream1: 2, Stream2: 1
// Stream1: 3, Stream2: 1
// ...
```

- Wacht tot alle streams **ten minste één keer** hebben geëmitteerd, geef dan de laatste waardecombinatie uit **telkens wanneer één van hen emitteert**.
- Omdat het in tupelvorm wordt geaccepteerd, is het type-veilig in TypeScript.

[🌐 RxJS Officiële Documentatie - `combineLatestWith`](https://rxjs.dev/api/operators/combineLatestWith)


## 💡 Typische gebruikspatronen

- **Realtime formuliervalidatie**: Combineer en valideer de laatste waarden van meerdere velden
- **Meerdere sensoren integratie**: Gelijktijdige weergave van gegevens met verschillende frequenties zoals temperatuur, vochtigheid, enz.
- **Gecombineerde zoekfilters**: Integreer categorieselectie en trefwoordinvoer
- **Live preview**: Realtime preview die meerdere configuratiewaarden combineert


## 🧠 Praktisch codevoorbeeld (met UI)

Voorbeeld van realtime kleur (RGB) wijziging met meerdere schuifregelaars.

```ts
import { fromEvent, combineLatest } from 'rxjs';
import { map, startWith, combineLatestWith } from 'rxjs';

// Bouw de UI
const container = document.createElement('div');
container.innerHTML = `
  <h3>combineLatestWith Praktisch voorbeeld: RGB Kleurenkiezer</h3>
  <div>
    <label>Rood: <input type="range" id="red" min="0" max="255" value="128"></label>
    <span id="red-value">128</span>
  </div>
  <div>
    <label>Groen: <input type="range" id="green" min="0" max="255" value="128"></label>
    <span id="green-value">128</span>
  </div>
  <div>
    <label>Blauw: <input type="range" id="blue" min="0" max="255" value="128"></label>
    <span id="blue-value">128</span>
  </div>
  <div id="preview" style="width: 200px; height: 100px; border: 1px solid #ccc; margin-top: 10px;"></div>
`;
document.body.appendChild(container);

// Verkrijg schuifregelaar-elementen
const redSlider = document.getElementById('red') as HTMLInputElement;
const greenSlider = document.getElementById('green') as HTMLInputElement;
const blueSlider = document.getElementById('blue') as HTMLInputElement;

const redValue = document.getElementById('red-value')!;
const greenValue = document.getElementById('green-value')!;
const blueValue = document.getElementById('blue-value')!;
const preview = document.getElementById('preview')!;

// Stream voor elke schuifregelaar
const red$ = fromEvent(redSlider, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value)),
  startWith(128)
);

const green$ = fromEvent(greenSlider, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value)),
  startWith(128)
);

const blue$ = fromEvent(blueSlider, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value)),
  startWith(128)
);

// ✅ Pipeable Operator versie - integreer anderen in hoofdstream
red$
  .pipe(
    combineLatestWith(green$, blue$)
  )
  .subscribe(([r, g, b]) => {
    // Update waardeweergave
    redValue.textContent = String(r);
    greenValue.textContent = String(g);
    blueValue.textContent = String(b);

    // Update preview achtergrondkleur
    preview.style.backgroundColor = `rgb(${r}, ${g}, ${b})`;
  });
```

- Het verplaatsen van een schuifregelaar zal **onmiddellijk** de preview bijwerken met de laatste RGB-waarden gecombineerd.
- Nadat alle schuifregelaars ten minste één keer zijn gemanipuleerd, wordt altijd de laatste combinatie weergegeven.


## 🔄 Verschil met Creation Function `combineLatest`

### Basisverschillen

| | `combineLatest` (Creation Function) | `combineLatestWith` (Pipeable Operator) |
|:---|:---|:---|
| **Gebruikslocatie** | Gebruikt als onafhankelijke functie | Gebruikt binnen `.pipe()` keten |
| **Syntaxis** | `combineLatest([obs1$, obs2$, obs3$])` | `obs1$.pipe(combineLatestWith(obs2$, obs3$))` |
| **Eerste stream** | Behandelt alle gelijk | Behandelt als hoofdstream |
| **Voordeel** | Eenvoudig en leesbaar | Gemakkelijk te combineren met andere operators |

### Specifieke gebruiksvoorbeelden

**Creation Function wordt aanbevolen voor alleen eenvoudige combinatie**

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map } from 'rxjs';

const width$ = fromEvent(window, 'resize').pipe(map(() => window.innerWidth));
const height$ = fromEvent(window, 'resize').pipe(map(() => window.innerHeight));

// Eenvoudig en leesbaar
combineLatest([width$, height$]).subscribe(([w, h]) => {
  console.log(`Venstergrootte: ${w} x ${h}`);
});
```

**Pipeable Operator wordt aanbevolen bij toevoegen van transformatieverwerking aan hoofdstream**

```ts
import { fromEvent, interval } from 'rxjs';
import { combineLatestWith, map, startWith, throttleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
const timer$ = interval(1000);

// ✅ Pipeable Operator versie - voltooid in één pipeline
clicks$
  .pipe(
    throttleTime(500),           // Voorkom snel klikken
    map(() => Date.now()),       // Converteren naar tijdstempel
    startWith(0),                // Initiële waarde instellen
    combineLatestWith(timer$),   // Integreren met timer
    map(([clickTime, tick]) => ({
      lastClick: clickTime,
      elapsed: tick
    }))
  )
  .subscribe(data => {
    console.log(`Laatste klik: ${data.lastClick}, Verstreken: ${data.elapsed} seconden`);
  });

// ❌ Creation Function versie - wordt uitgebreid
import { combineLatest } from 'rxjs';
combineLatest([
  clicks$.pipe(
    throttleTime(500),
    map(() => Date.now()),
    startWith(0)
  ),
  timer$
]).pipe(
  map(([clickTime, tick]) => ({
    lastClick: clickTime,
    elapsed: tick
  }))
).subscribe(data => {
  console.log(`Laatste klik: ${data.lastClick}, Verstreken: ${data.elapsed} seconden`);
});
```

### Samenvatting

- **`combineLatest`**: Optimaal voor eenvoudig combineren van meerdere streams
- **`combineLatestWith`**: Optimaal wanneer u andere streams wilt integreren terwijl u de hoofdstream transformeert of verwerkt


## ⚠️ Belangrijke opmerkingen

### Wacht tot alle streams ten minste één keer emitteren

Waarden worden pas uitgegeven als alle Observables ten minste één keer hebben geëmitteerd.

```ts
import { of, timer } from 'rxjs';
import { combineLatestWith } from 'rxjs';

of(1, 2, 3).pipe(
  combineLatestWith(
    timer(1000),  // Emitteert na 1 seconde
  )
).subscribe(console.log);
// Output: [3, 0]
// * Wacht tot timer$ emitteert, combineert dan met de laatste waarde (3) van of() op dat moment
```

### Pas op voor hoogfrequente updates

Als een van de streams vaak wordt bijgewerkt, zal het gecombineerde resultaat dienovereenkomstig vaak worden uitgegeven.

```ts
import { interval } from 'rxjs';
import { combineLatestWith, take } from 'rxjs';

interval(100).pipe(
  take(5),
  combineLatestWith(interval(1000).pipe(take(3)))
).subscribe(console.log);
// Output:
// [0, 0]
// [1, 0]
// [2, 0]
// [3, 0]
// [4, 0]
// [4, 1]
// [4, 2]
```

Controleer de updatefrequentie met `throttleTime` of `debounceTime` indien nodig.

```ts
import { fromEvent, interval } from 'rxjs';
import { combineLatestWith, throttleTime, map } from 'rxjs';

const mouseMoves$ = fromEvent(document, 'mousemove').pipe(
  throttleTime(100),  // Beperk elke 100ms
  map(e => ({ x: (e as MouseEvent).clientX, y: (e as MouseEvent).clientY }))
);

const timer$ = interval(1000);

mouseMoves$
  .pipe(combineLatestWith(timer$))
  .subscribe(([pos, tick]) => {
    console.log(`Positie: (${pos.x}, ${pos.y}), Tick: ${tick}`);
  });
```

### Foutafhandeling

Als een fout optreedt in een Observable, wordt de hele stream met een fout beëindigd.

```ts
import { throwError, interval } from 'rxjs';
import { combineLatestWith, take, catchError } from 'rxjs';
import { of } from 'rxjs';

interval(1000).pipe(
  take(2),
  combineLatestWith(
    throwError(() => new Error('Fout opgetreden')).pipe(
      catchError(err => of('Fout hersteld'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error('Fout:', err.message)
});
// Output: [1, 'Fout hersteld']
```


## 📚 Gerelateerde operators

- **[combineLatest](/nl/guide/creation-functions/combination/combineLatest)** - Creation Function versie
- **[zipWith](/nl/guide/operators/combination/zipWith)** - Paar corresponderende waarden (volgorde gegarandeerd)
- **[withLatestFrom](/nl/guide/operators/combination/withLatestFrom)** - Combineer alleen wanneer hoofdstream emitteert
