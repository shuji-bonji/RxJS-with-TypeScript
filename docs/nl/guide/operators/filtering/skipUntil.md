---
description: "De skipUntil operator slaat alle waarden van de oorspronkelijke Observable over totdat een andere Observable een waarde afgeeft, waarna de waarde als normaal wordt uitgevoerd. Dit is handig voor op tijd gebaseerde uitgestelde starts of nadat een specifieke gebeurtenis heeft plaatsgevonden."
---

# skipUntil - overslaan naar ontsteking

De operator ` skipUntil` slaat alle waarden van de oorspronkelijke Observable** over totdat de eerste waarde wordt afgegeven door de gespecificeerde Observable (kennisgevingstrigger). Na het tijdstip waarop de kennisgevingstrigger wordt afgegeven, worden de waarden zoals gebruikelijk uitgevoerd.

## 🔰 Basissyntaxis en gebruik

```ts
import { interval, timer } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500); // 0.5Waarde elke seconde uitgeven
const notifier$ = timer(2000); // 2Waarde na elke seconde uitgeven

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// Uitgang: 4, 5, 6, 7, 8, ...
// (eerste2tweede waarde 0, 1, 2, 3 worden overgeslagen)
```

**Bewerkingsstroom**:.
1. `bron$` geeft 0, 1, 2, 3 → alles overslaan
2. 2 seconden later geeft `melder$` een waarde uit
3. volgende `bron$` waarden (4, 5, 6, ...) worden zoals gewoonlijk uitgevoerd.

[🌐 Officiële RxJS documentatie - ` skipUntil`](https://rxjs.dev/api/operators/skipUntil)

## 🆚 Contrast met takeUntil

`skipUntil` en `takeUntil` hebben contrasterend gedrag.

```ts
import { interval, timer } from 'rxjs';
import { skipUntil, takeUntil } from 'rxjs';

const source$ = interval(500); // 0.5Waarde elke seconde uitgeven
const notifier$ = timer(2000); // 2Waarde na elke seconde uitgeven

// takeUntil: Waarde ophalen tot melding
source$.pipe(
  takeUntil(notifier$)
).subscribe(console.log);
// Uitgang: 0, 1, 2, 3(Stopt na2(stopt na 1,5 seconden)

// skipUntil: Waarden overslaan tot melding
source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// Uitgang: 4, 5, 6, 7, ...(Stopt na2(Begint na 1,5 seconden)
```

TABEL 10

## 💡 Typisch gebruikspatroon

1. **Begin met het verwerken van gegevens na authenticatie van de gebruiker**.

```ts
   import { interval, Subject } from 'rxjs';
   import { skipUntil } from 'rxjs';

   const authenticated$ = new Subject<void>();
   const dataStream$ = interval(1000);

   // Gegevens overslaan tot authenticatie is voltooid
   dataStream$.pipe(
     skipUntil(authenticated$)
   ).subscribe(data => {
     console.log(`Gegevensverwerking: ${data}`);
   });

   // 3Authenticatie voltooid na 2 seconden
   setTimeout(() => {
     console.log('Authenticatie voltooid！');
     authenticated$.next();
   }, 3000);
   // 3(Begint na seconde) "Gegevensverwerking: 3Gegevensverwerking: 4', 'Gegevensverwerking...en uitvoer
   ```

2. **Gebeurtenisverwerking start na voltooiing van initieel laden**
   ```ts
   import { fromEvent, BehaviorSubject } from 'rxjs';
   import { filter, skipUntil } from 'rxjs';

   const appReady$ = new BehaviorSubject<boolean>(false);
   const button = document.createElement('button');
   button.textContent = 'Klikken.';
   document.body.appendChild(button);

   const clicks$ = fromEvent(button, 'click');

   // Negeer klikken totdat de app klaar is
   clicks$.pipe(
     skipUntil(appReady$.pipe(filter(ready => ready)))
   ).subscribe(() => {
     console.log('Klik verwerkt');
   });

   // 2App klaar in seconden
   setTimeout(() => {
     console.log('App is klaar');
     appReady$.next(true);
   }, 2000);
   ```

3. **Op timer gebaseerde vertraging gestart**
   ```ts
   import { fromEvent, timer } from 'rxjs';
   import { skipUntil, scan } from 'rxjs';

   const button = document.createElement('button');
   button.textContent = 'Tellen';
   document.body.appendChild(button);

   const clicks$ = fromEvent(button, 'click');
   const startTime$ = timer(3000); // 3Seconden later

   // 3Kliks worden pas geteld nadat seconden zijn verstreken
   clicks$.pipe(
     skipUntil(startTime$),
     scan(count => count + 1, 0)
   ).subscribe(count => {
     console.log(`Tellen: ${count}`);
   });

   console.log('3Tellen begint na seconden...');
   ```

## 🧠 Praktisch codevoorbeeld (aftellen van spel)

Dit is een voorbeeld van het negeren van klikken tijdens het aftellen voordat het spel begint en het inschakelen van klikken nadat het aftellen is afgelopen.

```

ts.
import { fromEvent, timer, interval } from 'rxjs';
import { skipUntil, take, scan } from 'rxjs';

// UI-elementen maken
const container = document.createElement('div');.
document.body.appendChild(container);

const countdown = document.createElement('div');
countdown.style.fontSize = '24px';
countdown.style.marginBottom = '10px';
countdown.textContent = 'Aftellen bezig...'. ;
container.appendChild(countdown);

const button = document.createElement('button');
button.textContent = 'Klik! ;
button.disabled = true;
container.appendChild(button);

const scoreDisplay = document.createElement('div');
scoreDisplay.style.marginTop = '10px';
scoreDisplay.textContent = 'score: 0';
container.appendChild(scoreDisplay);

// Aftellen (3 seconden)
const countdownTimer$ = interval(1000).pipe(take(3));
countdownTimer$.subscribe({
  next: (n) => {
    countdown.textContent = `${3 - n} seconden tot start... `;
  },.
  complete: () => {
    countdown.textContent = `Het spel begint! ;
    knop.uitgeschakeld = onwaar;
  }
});

// Spelstartmelding
const gameStart$ = timer(3000);.

// Klikgebeurtenis (slaat over naar het begin van het spel)
const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  skipUntil(gameStart$),.
  scan(score => score + 10, 0)
).subscribe(score => {
  scoreDisplay.textContent = `score: ${score}`;
});

```

In deze code is het aftellen3seconden, worden klikken tijdens het aftellen genegeerd en worden alleen klikken na het aftellen weergegeven in de score.

## 🎯 skip Het verschil tussen skipUntil Verschil tussen

```

ts.
import { interval, timer } from 'rxjs';
import { skip, skipUntil } from 'rxjs';

const source$ = interval(500);.

// skip: sla de eerste N per getal over
bron$.pipe(
  skip(3)
).subscribe(console.log);
// uitvoer: 3, 4, 5, 6, ...

// skipUntil: overslaan totdat een andere Observable afgaat
source$.pipe(
  skipUntil(timer(1500))
).subscribe(console.log);.
// uitvoer: 3, 4, 5, 6, ... (hetzelfde resultaat, maar andere controlemethode)

```

| Operator | Voorwaarden voor overslaan | Gebruik |
|---|---|---|
| `skip(n)` | EerstenSla een aantal stukken over | Een vast aantal overslaan |
| `skipWhile(predicate)` | Overslaan terwijl aan voorwaarden is voldaan | Overslaan op basis van voorwaarden |
| `skipUntil(notifier$)` | Overslaan tot een anderObservableOverslaan tot een | Gebeurtenis/Overslaan op basis van tijd |

## 📋 Type-veilig gebruik

TypeScript Dit is een voorbeeld van een typeveilige implementatie die gebruik maakt van generics in

```

ts.
import { Observable, Subject, fromEvent } from 'rxjs';
import { skipUntil, map } from 'rxjs';

interface GameState {
  status: 'waiting' | 'ready' | 'playing' | 'finished';
}

interface ClickEvent {
  timestamp: getal; }
  x: getal;
  y: getal;
}

klasse Spel {
  private gameReady$ = new Subject();
  private state: GameState = { status: 'waiting' };.

  startGame(element: HTMLElement): Observable {
    const clicks$ = fromEvent\<MouseEvent>(element, 'click').pipe(
      map(event => ({
        timestamp: Date.now(),.
        x: event.clientX, event.
        y: event.clientY
      } as ClickEvent)),.
      skipUntil(this.gameReady$)
    );

    // Melding van gereedheid
    setTimeout() => {
      this.state = {status: 'klaar' };
      this.gameReady$.next();
      console.log('Spel klaar!') ;
    }, 2000);

    return clicks$;
  }
}

// Gebruiksvoorbeeld
spel = nieuw spel();
const canvas = document.createElement('div');
canvas.style.width = '300px';
canvas.style.height = '200px';
canvas.style.border = '1px solid black';
canvas.textContent = 'Klik hier';
document.body.appendChild(canvas);

game.startGame(canvas).subscribe(click => {
  console.log(`Klik positie: (${klik.x}, ${klik.y})`);
});

```

## 🔄 skipUntil Het verschil tussen takeUntil Combinatie van

Combineer beide als je alleen waarden wilt krijgen voor een specifieke tijdsperiode.

```

ts.
import { interval, timer } from 'rxjs';
import { skipUntil, takeUntil } from 'rxjs';

const source$ = interval(500);
const start$ = timer(2000); // start na 2 seconden
const stop$ = timer(5000); // stop na 5 seconden

source$.pipe(
  skipUntil(start$), // sla over tot na 2 seconden
  takeUntil(stop$); // stop na 5 seconden
).subscribe({
  next: console.log,.
  complete: () => console.log('complete')
});
// Uitvoer: 4, 5, 6, 7, 8, 9, voltooien.
// (alleen waarden tussen 2 en 5 seconden worden opgehaald)

```

**Tijdlijnen**:
```

0s 1s 2s 3s 4s 5s

```ts
import { interval, timer } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500); // 0.5Waarde elke seconde uitgeven
const notifier$ = timer(2000); // 2Waarde na elke seconde uitgeven

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// Uitgang: 4, 5, 6, 7, 8, ...
// (eerste2tweede waarde 0, 1, 2, 3 worden overgeslagen)
```

0 1 2 3 4 5 6 7 8 9 10
      ↑ omhoog omhoog omhoog omhoog
   SKIP start TAKE einde
   (vanaf 4) (tot 9)


## ⚠️ Een veelgemaakte fout

> [!IMPORTANT]
> `skipUntil` zijn meldingen Observable van de**Alleen het eerste afvuren**is geldig.2De tweede en volgende ontstekingen worden genegeerd.

### Vals: MeldingObservablewordt meer dan één keer geactiveerd.

```

ts
import { interval, Subject } from 'rxjs';
import { skipUntil } from 'rxjs';

const bron$ = interval(500);
const melder$ = nieuw Subject();

source$.pipe(
  skipUntil(kennisgever$)
).subscribe(console.log);

// ❌ Slecht voorbeeld: roep next meerdere keren aan, maar alleen de eerste heeft effect
setTimeout() => notifier$.next(), 1000);
setTimeout() => notifier$.next(), 2000); // dit is zinloos

```

### Correct.: Begrijp dat alleen de eerste afvuring geldig is.

```

ts.
import { interval, Subject } from 'rxjs';
import { skipUntil } from 'rxjs';

const bron$ = interval(500);
const melder$ = nieuw Subject();

source$.pipe(
  skipUntil(kennisgever$)
).subscribe(console.log);

// ✅ Goed voorbeeld: roep next maar één keer aan
setTimeout() => {
  console.log('Einde overslaan');
  notifier$.next();
  notifier$.complete(); // best practice om te completeren.
}, 1000);
```

## Samenvatting

### Wanneer skipUntil moet worden gebruikt.
- ✅ Als u de verwerking wilt starten nadat een specifieke gebeurtenis heeft plaatsgevonden
- ✅ Als u gebruikersbewerkingen wilt inschakelen nadat de initialisatie is voltooid
- ✅ Als u een op tijd gebaseerde uitgestelde start nodig hebt
- ✅ Als u de gegevensverwerking wilt starten nadat de authenticatie is voltooid

### In combinatie met takeUntil.
- ✅ Als u alleen waarden voor een bepaalde periode wilt krijgen (skipUntil + takeUntil)

### Opmerkingen.
- ⚠️ Alleen het eerste afvuren van de Observable is geldig.
- ⚠️ Als de Observable niet afgaat, worden alle waarden nog steeds overgeslagen.
- ⚠️ Abonnement wordt gehandhaafd totdat de oorspronkelijke stroom is voltooid

## Volgende stappen.

- **[skip](. /skip)** - leer hoe je de eerste N waarden kunt overslaan.
- **[take](. /take)** - leren hoe je de eerste N waarden krijgt.
- **[takeUntil](. /utility/takeUntil)** - leer hoe u waarden kunt nemen totdat een andere Observable afgaat.
- Filter](. /filter)** - leren filteren op basis van voorwaarden
- **[filtering-operator-praktische-gebruiksgevallen](. /practical-use-cases)** - leer echte use-cases
