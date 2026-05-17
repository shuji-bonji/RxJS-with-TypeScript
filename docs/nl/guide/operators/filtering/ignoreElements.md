---
description: "De ignoreElements operator is een RxJS filteroperator die alle waarden negeert en alleen voltooiingen en fouten doorlaat. Dit is handig wanneer je wacht tot het proces voltooid is."
---

# ignoreElements - alleen voltooiingen/fouten passeren

De `ignoreElements` operator **negeert alle waarden** uitgegeven door de bron Observable en alleen **completion en foutmeldingen** worden downstream doorgegeven.

## 🔰 Basissyntaxis en gebruik

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Waarde:', value), // Niet opgeroepen
  complete: () => console.log('Voltooid')
});
// Uitvoer: Voltooid
```

**Werking**:.
1. alle 1, 2, 3, 4 en 5 worden genegeerd
2. alleen voltooiingsmeldingen worden stroomafwaarts doorgegeven

[🌐 Officiële RxJS documentatie - `ignoreElements`](https://rxjs.dev/api/operators/ignoreElements)

## 💡 Typisch gebruikspatroon.

- **Wacht op voltooiing**: als je de waarde niet nodig hebt en alleen de voltooiing wilt weten.
- **Alleen neveneffecten uitvoeren**: neveneffecten uitvoeren met tap en waarden negeren
- **Fouten afhandelen**: als je alleen fouten wilt opvangen
- **Synchroniseren van sequenties**: wachten tot meerdere processen voltooid zijn

## Praktisch codevoorbeeld 1: Wachten op voltooiing van initialisatieproces

Dit is een voorbeeld van wachten tot meerdere initialisatieprocessen voltooid zijn.

```ts
import { from, forkJoin, of } from 'rxjs';
import { ignoreElements, tap, delay, concat } from 'rxjs';

// UIAangemaakt
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Applicatie initialisatie';
container.appendChild(title);

const statusArea = document.createElement('div');
statusArea.style.marginTop = '10px';
container.appendChild(statusArea);

const completeMessage = document.createElement('div');
completeMessage.style.marginTop = '10px';
completeMessage.style.padding = '10px';
completeMessage.style.display = 'none';
container.appendChild(completeMessage);

// Functie om statuslogboek toe te voegen
function addLog(message: string, color: string = 'black') {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
  log.style.color = color;
  statusArea.appendChild(log);
}

// Initialisatieproces1: Databaseverbinding
const initDatabase$ = from(['DBVerbinding maken met...', 'Tabel controleren...', 'DBGereed']).pipe(
  tap(msg => addLog(msg, 'blue')),
  delay(500),
  ignoreElements() // Waarden genegeerd, alleen voltooiing gemeld
);

// Initialisatieproces2: Configuratiebestand wordt gelezen
const loadConfig$ = from(['Configuratiebestand wordt gelezen...', 'Configuratieanalyse bezig...', 'Configuratie toepassing voltooid']).pipe(
  tap(msg => addLog(msg, 'green')),
  delay(700),
  ignoreElements()
);

// Initialisatieproces3: Authenticatie gebruiker
const authenticate$ = from(['Authenticatie-informatie wordt geverifieerd...', 'Tokenverificatie bezig...', 'Authenticatie voltooid']).pipe(
  tap(msg => addLog(msg, 'purple')),
  delay(600),
  ignoreElements()
);

// Alle initialisatieprocessen worden uitgevoerd.
addLog('Initialisatie gestart...', 'orange');

forkJoin([
  initDatabase$,
  loadConfig$,
  authenticate$
]).subscribe({
  complete: () => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#e8f5e9';
    completeMessage.style.color = 'green';
    completeMessage.style.fontWeight = 'bold';
    completeMessage.textContent = '✅ Alle initialisatie is voltooid.！De toepassing kan worden gestart.';
    addLog('Applicatie gestart', 'green');
  },
  error: err => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#ffebee';
    completeMessage.style.color = 'red';
    completeMessage.textContent = `❌ Fout bij initialisatie: ${err.message}`;
  }
});
```

- Er wordt een gedetailleerd logboek van elk initialisatieproces weergegeven, maar de waarden worden genegeerd.
- Als alle processen zijn voltooid, wordt er een voltooiingsbericht weergegeven.

## 🎯 Praktijkvoorbeeld 2: Wachten tot het uploaden van bestanden is voltooid

Dit is een voorbeeld van het weergeven van de uploadvoortgang van meerdere bestanden, maar alleen van de voltooiing.

```ts
import { from, of, concat } from 'rxjs';
import { ignoreElements, tap, delay, mergeMap } from 'rxjs';

// UIAangemaakt
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Bestand uploaden';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'Uploaden gestart';
container.appendChild(button);

const progressArea = document.createElement('div');
progressArea.style.marginTop = '10px';
container.appendChild(progressArea);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.display = 'none';
container.appendChild(result);

interface FileUpload {
  name: string;
  size: number;
}

const files: FileUpload[] = [
  { name: 'document.pdf', size: 2500 },
  { name: 'image.jpg', size: 1800 },
  { name: 'video.mp4', size: 5000 }
];

// Bestandsuploadproces (met voortgangsindicatie)
function uploadFile(file: FileUpload) {
  const fileDiv = document.createElement('div');
  fileDiv.style.marginTop = '5px';
  fileDiv.style.padding = '5px';
  fileDiv.style.border = '1px solid #ccc';
  progressArea.appendChild(fileDiv);

  const progressSteps = [0, 25, 50, 75, 100];

  return from(progressSteps).pipe(
    delay(200),
    tap(progress => {
      fileDiv.textContent = `📄 ${file.name} (${file.size}KB) - ${progress}%`;
      if (progress === 100) {
        fileDiv.style.backgroundColor = '#e8f5e9';
      }
    }),
    ignoreElements() // Voortgangswaarden genegeerd, alleen melding van voltooiing
  );
}

button.addEventListener('click', () => {
  button.disabled = true;
  progressArea.innerHTML = '';
  result.style.display = 'none';

  // Alle bestanden sequentieel geüpload
  from(files).pipe(
    mergeMap(file => uploadFile(file), 2) // Max.23 bestanden parallel
  ).subscribe({
    complete: () => {
      result.style.display = 'block';
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
      result.innerHTML = `
        <strong>✅ Uploaden voltooid</strong><br>
        ${files.length}Eén bestand is geüpload
      `;
      button.disabled = false;
    },
    error: err => {
      result.style.display = 'block';
      result.style.backgroundColor = '#ffebee';
      result.style.color = 'red';
      result.textContent = `❌ Fout: ${err.message}`;
      button.disabled = false;
    }
  });
});
```

- De voortgang van elk bestand wordt weergegeven, maar de voortgangswaarden zelf gaan niet stroomafwaarts.
- Er wordt een voltooiingsbericht weergegeven wanneer alle uploads zijn voltooid.

## Vergelijking met vergelijkbare operatoren

### ignoreElements vs filter() => false) vs take(0)

```ts
import { of } from 'rxjs';
import { ignoreElements, filter, take } from 'rxjs';

const source$ = of(1, 2, 3);

// ignoreElements: Negeer alle waarden, voltooiing wordt doorgegeven
source$.pipe(
  ignoreElements()
).subscribe({
  next: v => console.log('Waarde:', v),
  complete: () => console.log('ignoreElements: Voltooid')
});
// Uitvoer: ignoreElements: Voltooid

// filter(() => false): Filter alle waarden, voltooiing wordt doorgelaten
source$.pipe(
  filter(() => false)
).subscribe({
  next: v => console.log('Waarde:', v),
  complete: () => console.log('filter: Voltooid')
});
// Uitvoer: filter: Voltooid

// take(0): Onmiddellijk voltooid
source$.pipe(
  take(0)
).subscribe({
  next: v => console.log('Waarde:', v),
  complete: () => console.log('take(0): Voltooid')
});
// Uitvoer: take(0): Voltooid
```

TABEL 12

**Aanbevolen**: gebruik `ignoreElements()` als je opzettelijk alle waarden wilt negeren. De bedoeling van de code zal duidelijk zijn.

## 🔄 Foutmeldingen afhandelen.

`ignoreElements` negeert de waarden, maar **geeft foutmeldingen** door.

```ts
import { throwError, of, concat } from 'rxjs';
import { ignoreElements, delay } from 'rxjs';

const success$ = of(1, 2, 3).pipe(
  delay(100),
  ignoreElements()
);

const error$ = concat(
  of(1, 2, 3),
  throwError(() => new Error('Fout'))
).pipe(
  ignoreElements()
);

// Succesvol geval
success$.subscribe({
  next: v => console.log('Waarde:', v),
  complete: () => console.log('✅ Voltooid'),
  error: err => console.error('❌ Fout:', err.message)
});
// Uitvoer: ✅ Voltooid

// Fout
error$.subscribe({
  next: v => console.log('Waarde:', v),
  complete: () => console.log('✅ Voltooid'),
  error: err => console.error('❌ Fout:', err.message)
});
// Uitvoer: ❌ Fout: Fout
```

## ⚠️ Opmerkingen.

### 1. bijwerkingen worden uitgevoerd

`ignoreElements` negeert waarden, maar neveneffecten (bijv. `tap`) worden uitgevoerd.

```ts
import { of } from 'rxjs';
import { ignoreElements, tap } from 'rxjs';

of(1, 2, 3).pipe(
  tap(v => console.log('Bijwerkingen:', v)),
  ignoreElements()
).subscribe({
  next: v => console.log('Waarde:', v),
  complete: () => console.log('Voltooid')
});
// Uitvoer:
// Bijwerkingen: 1
// Bijwerkingen: 2
// Bijwerkingen: 3
// Voltooid
```

### 2. Gebruik met InfiniteObservable

Bij gebruik met Infinite Observable duurt het abonnement eeuwig omdat de voltooiing nooit komt.

```ts
import { interval } from 'rxjs';
import { ignoreElements, take } from 'rxjs';

// ❌ Slecht geval: Niet voltooid
interval(1000).pipe(
  ignoreElements()
).subscribe({
  complete: () => console.log('Voltooid') // Niet opgeroepen
});

// ✅ Goed voorbeeld: take Voltooid in
interval(1000).pipe(
  take(5),
  ignoreElements()
).subscribe({
  complete: () => console.log('Voltooid') // 5Afgeroepen na een seconde
});
```

### 3. Types in TypeScript

De terugkeerwaarde van `ignoreElements` is van het type `Observable<never>`.

```ts
import { Observable, of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const numbers$: Observable<number> = of(1, 2, 3);

// ignoreElements Het resultaat van Observable<never>
const result$: Observable<never> = numbers$.pipe(
  ignoreElements()
);

result$.subscribe({
  next: value => {
    // value is van het type never type, dus dit blok wordt niet uitgevoerd
    console.log(value);
  },
  complete: () => console.log('Alleen voltooien')
});
```

### 4. als voltooiing niet gegarandeerd is

Als de bron niet wordt voltooid, wordt `ignoreElements` ook niet voltooid.

```ts
import { NEVER } from 'rxjs';
import { ignoreElements } from 'rxjs';

// ❌ NEVERwordt niet voltooid en geeft ook geen foutmelding
NEVER.pipe(
  ignoreElements()
).subscribe({
  complete: () => console.log('Voltooid') // Niet opgeroepen
});
```

## Praktische combinatiepatronen

### Patroon 1: initialisatiesequentie

```ts
import { of, concat } from 'rxjs';
import { tap, ignoreElements, delay } from 'rxjs';

const initStep1$ = of('Step 1').pipe(
  tap(console.log),
  delay(1000),
  ignoreElements()
);

const initStep2$ = of('Step 2').pipe(
  tap(console.log),
  delay(1000),
  ignoreElements()
);

const initStep3$ = of('Step 3').pipe(
  tap(console.log),
  delay(1000),
  ignoreElements()
);

// Alle stappen worden sequentieel uitgevoerd
concat(initStep1$, initStep2$, initStep3$).subscribe({
  complete: () => console.log('✅ Alle initialisatie voltooid')
});
```

### Patroon 2: Opschoonproces

```ts
import { from, of } from 'rxjs';
import { tap, ignoreElements, mergeMap } from 'rxjs';

interface Resource {
  id: number;
  name: string;
}

const resources: Resource[] = [
  { id: 1, name: 'Database' },
  { id: 2, name: 'Cache' },
  { id: 3, name: 'Logger' }
];

from(resources).pipe(
  mergeMap(resource =>
    of(resource).pipe(
      tap(() => console.log(`🧹 ${resource.name} Opschonen bezig...`)),
      ignoreElements()
    )
  )
).subscribe({
  complete: () => console.log('✅ Alle bronnen opgeschoond')
});
```

## 📚 Gerelateerde operatoren.

- **[filter](. /filter)** - waarden filteren op basis van voorwaarden.
- **[take](. /take)** - alleen de eerste N waarden worden meegenomen.
- **[skip](. /skip)** - de eerste N waarden overslaan.
- **[tap](. /utility/tap)** - een nevenactie uitvoeren

## Samenvatting.

De `ignoreElements` operator negeert alle waarden en geeft alleen completies en fouten door.

- Ideaal als alleen een melding van voltooiing nodig is.
- ✅ Neveneffecten (TAP) worden uitgevoerd
- ✅ Foutmeldingen worden ook doorgegeven
- Duidelijkere intentie dan `filter() => false)`
- ⚠️ Oneindige observeerbaarheid wordt niet voltooid
- ⚠️ Het type terugkeerwaarde is `Observable<never>`.
- ⚠️ Waarde wordt volledig genegeerd, maar neveneffecten worden uitgevoerd
