---
description: auditTime is een RxJS filteroperator die wacht op een gespecificeerde tijd nadat een waarde is uitgegeven en de laatste waarde binnen die periode uitgeeft. Het is ideaal wanneer u periodiek de laatste status van hoogfrequente gebeurtenissen wilt bemonsteren, zoals scrollpositietracking, venstergrootte wijzigen en muisbewegingen.
titleTemplate: ':title | RxJS'
---

# auditTime - Geef laatste waarde uit na gespecificeerde tijd

De `auditTime` operator wacht op een **gespecificeerde tijd** nadat een waarde is uitgegeven en geeft de **laatste waarde** binnen die tijdsperiode uit. Vervolgens wacht het op de volgende waarde.


## 🔰 Basissyntax en gebruik

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Klik!'));
```

**Werkingsstroom**:
1. Eerste klik treedt op
2. Wacht 1 seconde (klikken tijdens deze tijd worden geregistreerd maar niet uitgegeven)
3. Geef de laatste klik uit na 1 seconde
4. Wacht op de volgende klik

[🌐 RxJS Officiële Documentatie - `auditTime`](https://rxjs.dev/api/operators/auditTime)


## 🆚 Contrast met throttleTime

`throttleTime` en `auditTime` zijn vergelijkbaar, maar geven verschillende waarden uit.

```ts
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

// throttleTime: Geef de eerste waarde uit
source$.pipe(
  throttleTime(1000)
).subscribe(console.log);
// Output: 0, 4, 8 (eerste waarde van elke periode)

// auditTime: Geef de laatste waarde uit
source$.pipe(
  auditTime(1000)
).subscribe(console.log);
// Output: 3, 6, 9 (laatste waarde van elke periode)
```

**Tijdlijnvergelijking**:
```
Bron:       0--1--2--3--4--5--6--7--8--9--|
            |        |        |
throttle:   0--------4--------8------------|
            (eerste) (eerste) (eerste)

audit:      -------3--------6--------9----|
                  (laatste) (laatste) (laatste)
```

| Operator | Uitvoerwaarde | Uitvoertiming | Gebruiksscenario |
|---|---|---|---|
| `throttleTime(ms)` | **Eerste** waarde binnen periode | Bij waarde-ontvangst | Onmiddellijke reactie nodig |
| `auditTime(ms)` | **Laatste** waarde binnen periode | Aan einde van periode | Laatste status nodig |
| `debounceTime(ms)` | **Laatste** waarde na stilte | Nadat invoer stopt | Wacht op invoervoltooiing |


## 💡 Typische gebruikspatronen

1. **Venstergrootte wijzigen optimalisatie**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime } from 'rxjs';

   fromEvent(window, 'resize').pipe(
     auditTime(200) // Haal laatste grootte elke 200ms op
   ).subscribe(() => {
     console.log(`Venstergrootte: ${window.innerWidth}x${window.innerHeight}`);
   });
   ```

2. **Scrollpositietracking**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime, map } from 'rxjs';

   fromEvent(window, 'scroll').pipe(
     auditTime(100),
     map(() => ({
       scrollY: window.scrollY,
       scrollX: window.scrollX
     }))
   ).subscribe(position => {
     console.log(`Scrollpositie: Y=${position.scrollY}, X=${position.scrollX}`);
   });
   ```


## 🎯 Verschil met debounceTime

`auditTime` en `debounceTime` geven beide **de laatste waarde uit**, maar de **timing is compleet anders**.

### Belangrijkste verschil

| Operator | Gedrag | Gebruiksscenario |
|---|---|---|
| `auditTime(ms)` | **Geeft altijd uit na ms** zodra waarde arriveert (zelfs als invoer doorgaat) | Periodiek bemonsteren gewenst |
| `debounceTime(ms)` | Geeft uit na ms **nadat invoer stopt** | Wacht op invoervoltooiing gewenst |

### Concreet voorbeeld: Verschil in zoekinvoer

```ts
import { fromEvent } from 'rxjs';
import { auditTime, debounceTime } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Voer zoekwoorden in';
document.body.appendChild(input);

// auditTime: Voer zoeken elke 300ms uit zelfs tijdens typen
fromEvent(input, 'input').pipe(
  auditTime(300)
).subscribe(() => {
  console.log('auditTime → Zoeken:', input.value);
});

// debounceTime: Voer zoeken 300ms uit nadat typen stopt
fromEvent(input, 'input').pipe(
  debounceTime(300)
).subscribe(() => {
  console.log('debounceTime → Zoeken:', input.value);
});
```

### Tijdlijnvisualisatie

Wanneer gebruiker "ab" → "abc" → "abcd" snel typt:

```
Invoergebeurtenissen:   a--b--c--d------------|
              ↓
auditTime:    ------c-----d----------|
            (na 300ms) (na 300ms)
            → Zoek "abc", zoek "abcd" (2 keer totaal)

debounceTime: --------------------d-|
                              (300ms na stoppen)
            → Zoek "abcd" (slechts 1 keer)
```

**Gemakkelijke herinnering**:
- **`auditTime`**: "Periodiek auditeren" → Controleer op regelmatige intervallen
- **`debounceTime`**: "Wacht tot het settelt (debounce)" → Wacht tot het rustig is


## 🧠 Praktisch codevoorbeeld (Muistracking)

Voorbeeld van muisbewegingen volgen en de laatste positie op regelmatige intervallen weergeven.

```ts
import { fromEvent } from 'rxjs';
import { auditTime, map } from 'rxjs';

// Maak UI-elementen
const container = document.createElement('div');
container.style.height = '300px';
container.style.border = '2px solid #3498db';
container.style.padding = '20px';
container.style.position = 'relative';
container.textContent = 'Beweeg uw muis binnen dit gebied';
document.body.appendChild(container);

const positionDisplay = document.createElement('div');
positionDisplay.style.marginTop = '10px';
positionDisplay.style.fontFamily = 'monospace';
document.body.appendChild(positionDisplay);

const dot = document.createElement('div');
dot.style.width = '10px';
dot.style.height = '10px';
dot.style.borderRadius = '50%';
dot.style.backgroundColor = '#e74c3c';
dot.style.position = 'absolute';
dot.style.display = 'none';
container.appendChild(dot);

// Muisbewegingsgebeurtenis
fromEvent<MouseEvent>(container, 'mousemove').pipe(
  map(event => {
    const rect = container.getBoundingClientRect();
    return {
      x: event.clientX - rect.left,
      y: event.clientY - rect.top
    };
  }),
  auditTime(100) // Haal laatste positie elke 100ms op
).subscribe(position => {
  positionDisplay.textContent = `Laatste positie (100ms interval): X=${position.x.toFixed(0)}, Y=${position.y.toFixed(0)}`;

  // Verplaats stip naar laatste positie
  dot.style.left = `${position.x - 5}px`;
  dot.style.top = `${position.y - 5}px`;
  dot.style.display = 'block';
});
```

Deze code haalt alleen de laatste positie elke 100ms op en toont deze, zelfs wanneer de muis frequent beweegt.


## 🎓 Samenvatting

### Wanneer auditTime gebruiken
- ✅ Wanneer u de laatste waarde op regelmatige intervallen nodig hebt
- ✅ Hoogfrequente gebeurtenissen zoals scroll, resize, muisbeweging
- ✅ Wanneer periodieke bemonstering nodig is
- ✅ Wanneer u de laatste status wilt weerspiegelen

### Wanneer throttleTime gebruiken
- ✅ Wanneer onmiddellijke reactie nodig is
- ✅ Wanneer u verwerking wilt starten met de eerste waarde
- ✅ Voorkom knop-mashing

### Wanneer debounceTime gebruiken
- ✅ Wanneer u wilt wachten op invoervoltooiing
- ✅ Zoeken, autocomplete
- ✅ Wacht tot gebruiker stopt met typen

### Opmerkingen
- ⚠️ `auditTime` geeft alleen de laatste waarde binnen de periode uit (tussenliggende waarden worden weggegooid)
- ⚠️ Als ingesteld op een kort interval, is het mogelijk niet erg effectief
- ⚠️ Afhankelijk van het gebruiksscenario kan `throttleTime` of `debounceTime` geschikter zijn


## 🚀 Volgende stappen

- **[throttleTime](/nl/guide/operators/filtering/throttleTime)** - Leer hoe u de eerste waarde doorlaat
- **[debounceTime](/nl/guide/operators/filtering/debounceTime)** - Leer hoe u waarden uitgeeft nadat invoer stopt
- **[filter](/nl/guide/operators/filtering/filter)** - Leer hoe u filtert op basis van voorwaarden
- **[Filteroperator praktische voorbeelden](/nl/guide/operators/filtering/practical-use-cases)** - Leer echte use cases
