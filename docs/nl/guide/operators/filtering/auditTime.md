---
description: "auditTime is een RxJS filteroperator die wacht op een gespecificeerde tijd wanneer een waarde wordt afgegeven en de laatste waarde binnen die periode uitvoert. Het wordt het best gebruikt als je periodiek de laatste status wilt samplen op hoogfrequente gebeurtenissen zoals scrollpositie bijhouden, venster vergroten of verkleinen, muisbeweging, enz. Het is belangrijk om het verschil te begrijpen tussen dit en throttleTime en debounceTime en ze op de juiste manier te gebruiken."
---

# auditTime - laatste waarde afgegeven na opgegeven tijd

De `auditTime` operator wacht tot een **gespecificeerde tijd** nadat een waarde is uitgegeven en voert de **laatste waarde** binnen die periode uit. Daarna wordt gewacht op de volgende waarde.

## 🔰 Basis syntaxis en gebruik

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Klik.！'));
```

**Bedieningsstroom**:.
1. eerste klik vindt plaats
2. wacht 1 seconde (klikken gedurende deze tijd worden geregistreerd, maar niet uitgevoerd)
3. de laatste klik wordt na 1 seconde uitgevoerd
4. wacht op de volgende klik

[🌐 RxJS officiële documentatie - `auditTime`](https://rxjs.dev/api/operators/auditTime)

## 🆚 Contrast met throttleTime

`throttleTime` en `auditTime` lijken op elkaar, maar verschillen in de waarden die ze uitvoeren.

```ts
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

// throttleTime: Eerste waarde uitvoeren
source$.pipe(
  throttleTime(1000)
).subscribe(console.log);
// Uitvoeren.: 0, 4, 8(eerste waarde van elke periode)

// auditTime: Uitvoer laatste waarde
source$.pipe(
  auditTime(1000)
).subscribe(console.log);
// Uitvoeren.: 3, 6, 9(laatste waarde van elke periode)
```

**Tijdlijnvergelijking**:.

```
Bron:     0--1--2--3--4--5--6--7--8--9--|
            |        |        |
throttle:   0--------4--------8------------|
            (Eerste)   (Eerste)   (Eerste)

audit:      -------3--------6--------9----|
                  (Laatste)   (Laatste)   (Laatste)
```

TABEL 12

## 💡 Typisch gebruikspatroon

1. **Het formaat van het venster aanpassen**.

```ts
   import { fromEvent } from 'rxjs';
   import { auditTime } from 'rxjs';

   fromEvent(window, 'resize').pipe(
     auditTime(200) // 200msVerkrijg de laatste grootte in het interval
   ).subscribe(() => {
     console.log(`Venstergrootte: ${window.innerWidth}x${window.innerHeight}`);
   });
   ```

2. **Scrollpositie bijhouden**
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

3. **Vloeiende sleepbeweging**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime, map, takeUntil, switchMap } from 'rxjs';

   // Sleepbare elementen maken
   const box = document.createElement('div');
   box.style.width = '100px';
   box.style.height = '100px';
   box.style.backgroundColor = '#3498db';
   box.style.position = 'absolute';
   box.style.cursor = 'move';
   box.style.left = '100px';
   box.style.top = '100px';
   box.textContent = 'Slepen';
   box.style.display = 'flex';
   box.style.alignItems = 'center';
   box.style.justifyContent = 'center';
   box.style.color = 'white';
   document.body.appendChild(box);

   const mouseDown$ = fromEvent<MouseEvent>(box, 'mousedown');
   const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove');
   const mouseUp$ = fromEvent<MouseEvent>(document, 'mouseup');

   // Slepen implementeren
   mouseDown$.pipe(
     switchMap(startEvent => {
       const startX = startEvent.clientX - box.offsetLeft;
       const startY = startEvent.clientY - box.offsetTop;

       return mouseMove$.pipe(
         auditTime(16), // Ongeveer.60FPS(zie ook16ms) om de positie bij te werken
         map(moveEvent => ({
           x: moveEvent.clientX - startX,
           y: moveEvent.clientY - startY
         })),
         takeUntil(mouseUp$)
       );
     })
   ).subscribe(position => {
     box.style.left = `${position.x}px`;
     box.style.top = `${position.y}px`;
   });
   ```

## 🧠 Praktisch codevoorbeeld (muis volgen)

Dit voorbeeld houdt muisbewegingen bij en geeft regelmatig de laatste positie weer.

```

ts.
import { fromEvent } from 'rxjs';
import { auditTime, map } from 'rxjs';

// UI-elementen maken
const container = document.createElement('div');.
container.style.height = '300px';
container.style.border = '2px solid #3498db';
container.style.padding = '20px';
container.style.position = 'relative';
container.textContent = 'Beweeg de muis binnen dit gebied';
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
dot.style.position = 'absoluut';
dot.style.display = 'none';
container.appendChild(dot);

// Muisbewegingsgebeurtenis
fromEvent<MouseEvent>(container, 'mousemove').pipe(
  map(event => {
    const rect = container.getBoundingClientRect();
    return {
      x: event.clientX - rect.left,.
      y: event.clientY - rect.top
    };
  }),
  auditTime(100) // Elke 100 ms de laatste positie ophalen
).subscribe(positie => {
  positionDisplay.textContent = `Laatste positie (elke 100 ms): X=${position.x.toFixed(0)}, Y=${position.y.toFixed(0)}`;

  // Verplaats dot naar laatste positie
  dot.style.left = `${position.x - 5}px`;
  dot.style.top = `${position.y - 5}px`;
  dot.style.display = 'block';
});

```

Deze code zal alleen de laatste positie ophalen en weergeven telkens als de muis wordt verplaatst, zelfs als de muis vaak wordt verplaatst,100msDe code haalt alleen de laatste positie op en geeft deze weer bij elke muisbeweging.

## 🎯 debounceTime Verschillen tussen

`auditTime` en `debounceTime` is dat**beide de laatste waarde uitvoeren**maar de**De timing is compleet anders**de laatste waarde wordt uitgevoerd.

### Het doorslaggevende verschil

| Operator | operatie | gebruik van het systeem op verschillende manieren |
|---|---|---|
| `auditTime(ms)` | Wanneer een waarde binnenkomt**msAltijd uitvoer na**(zelfs als de invoer doorgaat) | Periodieke bemonstering |
| `debounceTime(ms)` | **Nadat de invoer is gestopt**msUitvoeren na | Wachten op voltooiing van invoer |

### Specifieke voorbeelden：Verschillen in zoekinvoer

```

ts.
import { fromEvent } from 'rxjs';
import { auditTime, debounceTime } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Zoekwoord invoer';
document.body.appendChild(input);

// auditTime: zoekactie elke 300 ms uitvoeren, zelfs tijdens invoer
fromEvent(input, 'input').pipe(
  auditTime(300)
).subscribe() => {
  console.log('auditTime → Zoeken:', input.value);
});

// debounceTime: wacht 300ms nadat invoer stopt, voer dan zoeken uit
fromEvent(input, 'input').pipe(
  debounceTime(300)
).subscribe() => {
  console.log('debounceTime → Zoeken:', input.value);
});

```

### Verschillen in tijdlijn

Verschil te zien wanneer een gebruiker klikt op "ab'→'abc'→'abcd' bij snel typen:

```

Invoerevent: a-b--c--d------------|
              ↓
auditTime: ------c-----d----------|
            (na 300 ms) (na 300 ms)
            → Zoek naar 'abc', zoek naar 'abcd' (in totaal 2 keer)

debounceTime: --------------------d-|
                              (300 ms na stop)
            → Zoek naar 'abcd' (in totaal slechts één keer)

```

**Gemakkelijk te onthouden**:
- **`auditTime`**: 'Regelmatig gecontroleerd (audit)"→ 'Regelmatig controleren'
- **`debounceTime`**: 'Wacht tot het rustig is (...)'.debounceWacht tot het rustig is.→ 'Wacht tot het rustig is'

### Praktisch gebruik

```

ts.
// ✅ auditTime indien van toepassing
// - De scrollpositie bijhouden (we willen deze periodiek krijgen, zelfs als we de hele tijd scrollen)
fromEvent(window, 'scroll').pipe(
  auditTime(100) // elke 100 ms de laatste positie krijgen
).subscribe(/* ... */);

// ✅ als debounceTime geschikt is.
// - zoekvak (we willen zoeken nadat de invoer is voltooid)
fromEvent(searchInput, 'input').pipe(
  debounceTime(300) // wacht 300ms nadat invoer stopt
).subscribe(/* ... */);

```

## 📋 Type-veilig gebruik

TypeScript Dit is een voorbeeld van een typeveilige implementatie die gebruik maakt van generics in

```

ts.
import { Observable, fromEvent } from 'rxjs';
import { auditTime, map } from 'rxjs';

interface MousePosition {
  x: getal;
  y: getal;
  timestamp: getal; }
}

functie trackMousePosition(
  element: HTMLElement,.
  intervalMs: getal
): waarneembaar<muispositie> {
  return fromEvent<MouseEvent>(element, 'mousemove').pipe(
    auditTime(intervalMs),.
    map(event => ({
      x: event.clientX, event.
      y: event.clientY,.
      timestamp: Date.now())
    } als MuisPositie))
  );
}

// Gebruiksvoorbeeld
const canvas = document.createElement('div');
canvas.style.width = '400px';
canvas.style.height = '300px';
canvas.style.border = '1px solid black';
document.body.appendChild(canvas);

trackMousePosition(canvas, 200).subscribe(position => {
  console.log(`Positie: (${position.x}, ${position.y}) op ${position.timestamp}`);
});

```

## 🔄 auditTime en throttleTime Combinatie van

In bepaalde scenario's kunnen beide worden gecombineerd.

```

ts.
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(100).pipe(take(50));.

// volgorde van throttleTime → auditTime
bron$.pipe(
  throttleTime(1000), // geef de eerste waarde elke seconde door
  auditTime(500) // wacht dan 500ms en voer de laatste waarde uit
).subscribe(console.log);.

```

## ⚠️ Een veelgemaakte fout

> [!WARNING]
> `auditTime` en `debounceTime` zijn verschillend in gedrag. Zoekinvoer, bijvoorbeeld, waarbij de gebruiker**Wachten tot de gebruiker stopt**In sommige gevallen, zoals bij zoekinvoer, moet u `debounceTime` om te wachten tot de gebruiker stopt met typen, bijvoorbeeld voor zoekinvoer.`auditTime` waarden afgeeft met regelmatige tussenpozen tijdens de invoer.

### Als er een foutieve: auditTime en debounceTime verwarren de

```

ts.
import { fromEvent } from 'rxjs';
import { auditTime } van 'rxjs';

// Maak een invoerveld voor zoeken
const input = document.createElement('input');.
input.type = 'tekst';
input.placeholder = "Zoeken... ;
document.body.appendChild(input);

// ❌ Slecht voorbeeld: gebruik auditTime voor zoekinvoer
fromEvent(input, 'input').pipe(
  auditTime(300) // zoeken wordt elke 300ms uitgevoerd tijdens het invoeren
).subscribe() => {
  console.log('Zoekopdracht uitgevoerd');
});

```

### juiste: debounceTime gebruik de

```

ts.
import { fromEvent } from 'rxjs';
import { debounceTime } van 'rxjs';

// Maak een zoekinvoerveld
const input = document.createElement('input');.
input.type = 'tekst';
input.placeholder = "Zoeken... ;
document.body.appendChild(input);

// ✅ Goed voorbeeld: gebruik debounceTime voor zoekinvoer
fromEvent(input, 'input').pipe(
  debounceTime(300) // Wacht 300ms nadat input stopt voordat je gaat zoeken
).subscribe() => {
  console.log('Zoekopdracht uitgevoerd', input.value);
});
```

## Samenvatting

### Wanneer auditTime moet worden gebruikt.
- ✅ Wanneer regelmatig bijgewerkte waarden nodig zijn
- ✅ Hoogfrequente gebeurtenissen zoals scrollen, wijzigen van grootte, muisbeweging
- ✅ Wanneer periodieke bemonstering vereist is
- ✅ Wanneer je de laatste status wilt weergeven.

### Wanneer throttleTime moet worden gebruikt.
- ✅ Als een onmiddellijke respons vereist is
- ✅ Als je de verwerking wilt starten met de eerste waarde
- ✅ Voorkomen van knoppen indrukken

### Wanneer debounceTime gebruiken.
- ✅ Als u wilt wachten op voltooiing van invoer
- ✅ Zoeken, automatisch aanvullen
- ✅ Wachten tot de gebruiker stopt met typen.

### Opmerkingen.
- ⚠️ `auditTime` voert alleen de laatste waarde in de periode uit (tussenliggende waarden worden genegeerd)
- ⚠️ Niet erg effectief indien ingesteld voor korte intervallen
- ⚠️ `throttleTime` of `debounceTime` kunnen geschikter zijn, afhankelijk van de toepassing

## Volgende stappen.

- **[throttleTime](. /throttleTime)** - leer hoe je de eerste waarde doorgeeft.
- **[debounceTime](. /debounceTime)** - leer hoe je waarden doorgeeft nadat de invoer stopt.
- **[filter](. /filter)** - leer filteren op basis van voorwaarden
- **[filtering-operator-praktische-gebruiksgevallen](. /practical-use-cases)** - leer echte use-cases gebruiken
