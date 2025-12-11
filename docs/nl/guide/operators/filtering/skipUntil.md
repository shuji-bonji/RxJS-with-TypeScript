---
description: De skipUntil operator slaat alle waarden van de originele Observable over tot een andere Observable een waarde uitgeeft, en geeft dan waarden normaal uit. Het is nuttig voor tijdgebaseerde vertraagde start of verwerking nadat een specifieke gebeurtenis optreedt.
---

# skipUntil - Sla over tot andere Observable vuurt

De `skipUntil` operator **slaat alle waarden van de bron Observable over** tot een gespecificeerde Observable (meldingstrigger) zijn eerste waarde uitgeeft. Nadat de meldingstrigger heeft geëmitteerd, worden volgende waarden normaal uitgegeven.


## 🔰 Basissyntax en gebruik

```ts
import { interval, timer } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500); // Geef waarde elke 0,5 seconden uit
const notifier$ = timer(2000); // Geef waarde uit na 2 seconden

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// Output: 4, 5, 6, 7, 8, ...
// (Eerste 2 seconden waarden 0, 1, 2, 3 worden overgeslagen)
```

**Werkingsstroom**:
1. `source$` geeft 0, 1, 2, 3 uit → allemaal overgeslagen
2. Na 2 seconden geeft `notifier$` een waarde uit
3. Volgende `source$` waarden (4, 5, 6, ...) worden normaal uitgegeven

[🌐 RxJS Officiële Documentatie - `skipUntil`](https://rxjs.dev/api/operators/skipUntil)


## 🆚 Contrast met takeUntil

`skipUntil` en `takeUntil` hebben contrasterend gedrag.

```ts
import { interval, timer } from 'rxjs';
import { skipUntil, takeUntil } from 'rxjs';

const source$ = interval(500); // Geef waarde elke 0,5 seconden uit
const notifier$ = timer(2000); // Geef waarde uit na 2 seconden

// takeUntil: Neem waarden tot melding
source$.pipe(
  takeUntil(notifier$)
).subscribe(console.log);
// Output: 0, 1, 2, 3 (stopt na 2 seconden)

// skipUntil: Sla waarden over tot melding
source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// Output: 4, 5, 6, 7, ... (begint na 2 seconden)
```

| Operator | Gedrag | Voltooiingstiming |
|---|---|---|
| `takeUntil(notifier$)` | **Neem** waarden tot melding | Automatisch voltooien bij melding |
| `skipUntil(notifier$)` | **Sla** waarden over tot melding | Wanneer bronstream voltooit |


## 💡 Typische gebruikspatronen

1. **Start dataverwerking na gebruikersauthenticatie**
   ```ts
   import { interval, Subject } from 'rxjs';
   import { skipUntil } from 'rxjs';

   const authenticated$ = new Subject<void>();
   const dataStream$ = interval(1000);

   // Sla data over tot authenticatie voltooid is
   dataStream$.pipe(
     skipUntil(authenticated$)
   ).subscribe(data => {
     console.log(`Data verwerken: ${data}`);
   });

   // Authenticatie voltooit na 3 seconden
   setTimeout(() => {
     console.log('Authenticatie voltooid!');
     authenticated$.next();
   }, 3000);
   // Na 3 seconden, geeft "Data verwerken: 3", "Data verwerken: 4", ... uit
   ```

2. **Start gebeurtenisverwerking nadat initiële lading voltooid is**
   ```ts
   import { fromEvent, BehaviorSubject } from 'rxjs';
   import { filter, skipUntil } from 'rxjs';

   const appReady$ = new BehaviorSubject<boolean>(false);
   const button = document.createElement('button');
   button.textContent = 'Klik';
   document.body.appendChild(button);

   const clicks$ = fromEvent(button, 'click');

   // Negeer klikken tot app gereed is
   clicks$.pipe(
     skipUntil(appReady$.pipe(filter(ready => ready)))
   ).subscribe(() => {
     console.log('Klik verwerkt');
   });

   // App gereed na 2 seconden
   setTimeout(() => {
     console.log('App gereed');
     appReady$.next(true);
   }, 2000);
   ```


## 🧠 Praktisch codevoorbeeld (Game aftellen)

Voorbeeld van het negeren van klikken tijdens het aftellen voordat het spel begint en het inschakelen van klikken nadat het aftellen eindigt.

```ts
import { fromEvent, timer, interval } from 'rxjs';
import { skipUntil, take, scan } from 'rxjs';

// Maak UI-elementen
const container = document.createElement('div');
document.body.appendChild(container);

const countdown = document.createElement('div');
countdown.style.fontSize = '24px';
countdown.style.marginBottom = '10px';
countdown.textContent = 'Aftellen...';
container.appendChild(countdown);

const button = document.createElement('button');
button.textContent = 'Klik!';
button.disabled = true;
container.appendChild(button);

const scoreDisplay = document.createElement('div');
scoreDisplay.style.marginTop = '10px';
scoreDisplay.textContent = 'Score: 0';
container.appendChild(scoreDisplay);

// Aftellen (3 seconden)
const countdownTimer$ = interval(1000).pipe(take(3));
countdownTimer$.subscribe({
  next: (n) => {
    countdown.textContent = `Start over ${3 - n} seconden...`;
  },
  complete: () => {
    countdown.textContent = 'Game Start!';
    button.disabled = false;
  }
});

// Game startmelding
const gameStart$ = timer(3000);

// Klikgebeurtenissen (sla over tot game start)
const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  skipUntil(gameStart$),
  scan(score => score + 10, 0)
).subscribe(score => {
  scoreDisplay.textContent = `Score: ${score}`;
});
```

In deze code worden klikken genegeerd tijdens het 3-seconden aftellen, en alleen klikken na het aftellen worden weerspiegeld in de score.


## 🎯 Verschil tussen skip en skipUntil

```ts
import { interval, timer } from 'rxjs';
import { skip, skipUntil } from 'rxjs';

const source$ = interval(500);

// skip: Sla eerste N waarden over op aantal
source$.pipe(
  skip(3)
).subscribe(console.log);
// Output: 3, 4, 5, 6, ...

// skipUntil: Sla over tot andere Observable vuurt
source$.pipe(
  skipUntil(timer(1500))
).subscribe(console.log);
// Output: 3, 4, 5, 6, ... (zelfde resultaat, maar andere controlemethode)
```

| Operator | Overslaanvoorwaarde | Gebruiksscenario |
|---|---|---|
| `skip(n)` | Sla eerste n over op aantal | Vast aantal overslaan |
| `skipWhile(predicate)` | Sla over terwijl voorwaarde voldaan is | Voorwaarde-gebaseerd overslaan |
| `skipUntil(notifier$)` | Sla over tot andere Observable vuurt | Gebeurtenis/tijdgebaseerd overslaan |


## 🔄 Combineren van skipUntil en takeUntil

Om waarden alleen voor een specifieke periode te nemen, combineer beide.

```ts
import { interval, timer } from 'rxjs';
import { skipUntil, takeUntil } from 'rxjs';

const source$ = interval(500);
const start$ = timer(2000); // Start na 2 seconden
const stop$ = timer(5000);  // Stop na 5 seconden

source$.pipe(
  skipUntil(start$), // Sla over tot 2 seconden
  takeUntil(stop$)   // Stop op 5 seconden
).subscribe({
  next: console.log,
  complete: () => console.log('Voltooid')
});
// Output: 4, 5, 6, 7, 8, 9, Voltooid
// (Alleen waarden tussen 2-5 seconden)
```

**Tijdlijn**:
```
0s    1s    2s    3s    4s    5s
|-----|-----|-----|-----|-----|
0  1  2  3  4  5  6  7  8  9  10
      ↑           ↑
   skip start  take einde
   (vanaf 4)   (tot 9)
```


## ⚠️ Veelgemaakte fouten

> [!IMPORTANT]
> `skipUntil` is alleen geldig voor de **eerste emissie** van de meldings-Observable. Tweede en volgende emissies worden genegeerd.

### Fout: Meldings-Observable vuurt meerdere keren

```ts
import { interval, Subject } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500);
const notifier$ = new Subject<void>();

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);

// ❌ Slecht voorbeeld: next meerdere keren aanroepen werkt maar één keer
setTimeout(() => notifier$.next(), 1000);
setTimeout(() => notifier$.next(), 2000); // Dit is betekenisloos
```

### Correct: Begrijp dat alleen eerste emissie geldig is

```ts
import { interval, Subject } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500);
const notifier$ = new Subject<void>();

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);

// ✅ Goed voorbeeld: Roep next slechts één keer aan
setTimeout(() => {
  console.log('Overslaan beëindigd');
  notifier$.next();
  notifier$.complete(); // Best practice om te voltooien
}, 1000);
```


## 🎓 Samenvatting

### Wanneer skipUntil gebruiken
- ✅ Wanneer u wilt beginnen met verwerken nadat een specifieke gebeurtenis optreedt
- ✅ Wanneer u gebruikersoperaties wilt inschakelen nadat initialisatie voltooid is
- ✅ Wanneer tijdgebaseerde vertraagde start nodig is
- ✅ Wanneer u dataverwerking wilt starten nadat authenticatie voltooid is

### Combinatie met takeUntil
- ✅ Wanneer u waarden alleen voor een specifieke periode wilt nemen (skipUntil + takeUntil)

### Opmerkingen
- ⚠️ Alleen de eerste emissie van de meldings-Observable is geldig
- ⚠️ Als de meldings-Observable niet emitteert, blijven alle waarden overgeslagen
- ⚠️ Abonnement wordt behouden tot de bronstream voltooit


## 🚀 Volgende stappen

- **[skip](/nl/guide/operators/filtering/skip)** - Leer hoe u eerste N waarden overslaat
- **[take](/nl/guide/operators/filtering/take)** - Leer hoe u eerste N waarden neemt
- **[takeUntil](../utility/takeUntil)** - Leer hoe u waarden neemt tot andere Observable vuurt
- **[filter](/nl/guide/operators/filtering/filter)** - Leer hoe u filtert op basis van voorwaarden
- **[Filteroperator praktische voorbeelden](/nl/guide/operators/filtering/practical-use-cases)** - Leer echte use cases
