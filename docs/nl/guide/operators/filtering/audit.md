---
description: "De controle operator is een RxJS filter operator die alleen de laatste waarde afgeeft binnen de periode die wordt gecontroleerd door de aangepaste Observable. Het is ideaal voor dynamische timingcontrole."
---

# audit - laatste waarde van controleperiode afgegeven

De `audit` operator wacht tot een aangepaste Observable een waarde afgeeft en geeft de **laatste waarde** afgegeven door de bron binnen die periode.
Terwijl `auditTime` wordt gecontroleerd door een vaste tijd, staat `audit` **controle van de periode** toe met een dynamische Observable.

## 🔰 Basis syntaxis en gebruik

```ts
import { fromEvent, interval } from 'rxjs';
import { audit } from 'rxjs';

// Klikgebeurtenis
const clicks$ = fromEvent(document, 'click');

// 1Afzonderlijke tijdsperioden elke seconde
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('Klik is opgenomen');
});
```

- Wanneer er een klik plaatsvindt, begint er een periode van één seconde.
- Alleen de laatste klik van die periode van 1 seconde wordt uitgegeven.
- Na één seconde begint de volgende periode.

[🌐 Officiële RxJS documentatie - `audit`](https://rxjs.dev/api/operators/audit)

> [!WARNING] Let op in productiecode

> Het bovenstaande voorbeeld laat het uitschrijven van `fromEvent` weg voor de eenvoud van de uitleg. Gebruik in echte code `takeUntil(destroy$)`, `take(N)` of `Subscription.unsubscribe()` om de levenscyclus expliciet te beheren. Meer informatie: [Moeilijkheden overwinnen: levenscyclusbeheer](/nl/guide/overcoming-difficulties/lifecycle-management.md)

## 💡 Typische gebruikspatronen

- Dynamische intervalsampling**: duur aanpassen aan belasting.
- **Dynamische intervalsampling**: duur aanpassen op basis van belasting.
- **Adaptieve gebeurtenisbeperking**: contextgevoelige uitdunning

## Verschillen met auditTime

| Beheerder. | Periodecontrole | Gebruikscasus. |
|---|---|---|
| `auditTime`. | Vaste tijd (milliseconden) | Eenvoudige tijdsgebaseerde controle |
| `audit`. | **Aangepaste waarneembaar** | **Dynamische periodecontrole**. |

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, auditTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// auditTime - Vast1seconden
clicks$.pipe(
  auditTime(1000)
).subscribe(() => console.log('Vast1seconden'));

// audit - Dynamische periode
let period = 1000;
clicks$.pipe(
  audit(() => {
    period = Math.random() * 2000; // 0~.2Willekeurige periode van seconden
    return timer(period);
  })
).subscribe(() => console.log(`Dynamische periode: ${period}ms`));
```

## Praktisch codevoorbeeld 1: Op belasting gebaseerde dynamische bemonstering

Dit is een voorbeeld van het aanpassen van het bemonsteringsinterval aan de systeembelasting.

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map } from 'rxjs';

// UIAanmaak
const output = document.createElement('div');
output.innerHTML = '<h3>Dynamische bemonstering</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Belasting wijzigen';
document.body.appendChild(button);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
output.appendChild(statusDiv);

const logDiv = document.createElement('div');
logDiv.style.marginTop = '10px';
logDiv.style.maxHeight = '200px';
logDiv.style.overflow = 'auto';
output.appendChild(logDiv);

// Belastingsniveau (0: Lage belasting,1: Gemiddelde belasting,2: Hoge belasting)
let loadLevel = 0;

fromEvent(button, 'click').subscribe(() => {
  loadLevel = (loadLevel + 1) % 3;
  const levels = ['Lage belasting', 'Gemiddelde belasting', 'Hoge belasting'];
  statusDiv.textContent = `Huidige belasting: ${levels[loadLevel]}`;
});

// Gebeurtenis muisbeweging
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  audit(() => {
    // Duur afhankelijk van belasting
    const periods = [2000, 1000, 500]; // Lage belasting→Lange duur, hoge belasting→Korte duur
    return timer(periods[loadLevel]);
  }),
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe(pos => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Muispositie: (${pos.x}, ${pos.y})`;
  logDiv.insertBefore(log, logDiv.firstChild);

  // Max.10Weergeven tot
  while (logDiv.children.length > 10) {
    logDiv.removeChild(logDiv.lastChild!);
  }
});
```

- Gedundeerd tot intervallen van 2 s wanneer de belasting laag is (energiebesparende modus)
- Fijne bemonstering met intervallen van 500 ms wanneer de belasting hoog is.
- De periode kan dynamisch worden aangepast aan de belasting.

## Praktisch codevoorbeeld 2: Periodebesturing gebaseerd op andere streams

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map, startWith } from 'rxjs';

// UIAanmaak
const container = document.createElement('div');
document.body.appendChild(container);

const slider = document.createElement('input');
slider.type = 'range';
slider.min = '100';
slider.max = '2000';
slider.value = '1000';
container.appendChild(document.createTextNode('Interval: '));
container.appendChild(slider);

const intervalDisplay = document.createElement('span');
intervalDisplay.textContent = ' 1000ms';
container.appendChild(intervalDisplay);

const output = document.createElement('div');
output.style.marginTop = '10px';
container.appendChild(output);

// Waarden schuifregelaar bewaken
const sliderValue$ = fromEvent(slider, 'input').pipe(
  map(() => Number(slider.value)),
  startWith(1000)
);

sliderValue$.subscribe(value => {
  intervalDisplay.textContent = ` ${value}ms`;
});

// Klikgebeurtenis
const clicks$ = fromEvent(document, 'click');

let currentInterval = 1000;

// Schuifwaarden bijwerken
sliderValue$.subscribe(value => {
  currentInterval = value;
});

// Klik opauditGecontroleerd door
clicks$.pipe(
  audit(() => timer(currentInterval))
).subscribe(() => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Klik record (interval: ${currentInterval}ms)`;
  output.insertBefore(log, output.firstChild);
});
```

## ⚠️ Opmerkingen.

### 1. de eerste waarde wordt niet onmiddellijk afgegeven

Nadat de `audit` de eerste waarde heeft ontvangen, wacht hij tot het einde van de periode.

```ts
import { interval, timer } from 'rxjs';
import { audit, take } from 'rxjs';

interval(100).pipe(
  audit(() => timer(1000)),
  take(3)
).subscribe(val => {
  console.log(val);
});
// Uitvoer:
// 9  (1Seconden later,0~.9Laatste waarde van)
// 19 (2Seconden later,10~.19Laatste waarde van)
// 29 (3Seconden later,20~.29Laatste waarde van)
```

### 2. De duration Observable wordt elke keer opnieuw gegenereerd.

Functies die worden doorgegeven aan `audit` **moeten elke keer een nieuwe waarneembare waarde teruggeven**.

```ts
// ❌ Slecht voorbeeld: Als dezelfdeObservableinstantie wordt gebruikt en opnieuw wordt gebruikt
const duration$ = timer(1000);
source$.pipe(
  audit(() => duration$) // 2Werkt niet na de tweede keer
).subscribe();

// ✅ Goed voorbeeld: Elke keer een nieuwe instantie aanmakenObservableGenereer een
source$.pipe(
  audit(() => timer(1000))
).subscribe();
```

### 3. geheugen en prestaties

Het gebruik van `audit` op streams waar vaak waarden worden uitgegeven verbruikt geheugen.

```ts
import { interval, timer } from 'rxjs';
import { audit } from 'rxjs';

// Snelle stream (10msper seconde)
interval(10).pipe(
  audit(() => timer(1000)) // 1Elke seconde bemonsteren
).subscribe();
// 1per seconde100Waarden worden in het geheugen opgeslagen en alleen de laatste1Alleen de laatste wordt uitgegeven
```

## Vergelijking met vergelijkbare operatoren

TABEL 10

```ts
import { fromEvent, interval, timer } from 'rxjs';
import { audit, throttle, debounce, sample } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// audit: 1Laatste klik in seconden
clicks$.pipe(
  audit(() => timer(1000))
).subscribe(() => console.log('audit: Laatste klik'));

// throttle: 1Eerste klik in seconden
clicks$.pipe(
  throttle(() => timer(1000))
).subscribe(() => console.log('throttle: Eerste'));

// debounce: Nadat de klik is gestopt1Seconden na
clicks$.pipe(
  debounce(() => timer(1000))
).subscribe(() => console.log('debounce: Na stop'));

// sample: 1Elke seconde bemonsteren
clicks$.pipe(
  sample(interval(1000))
).subscribe(() => console.log('sample: Periodiek'));
```

## 📚 Verwante operatoren.

- **[auditTime](. /auditTime)** - bestuurd door vaste tijd (vereenvoudigde versie van `audit`).
- **[throttle](. /throttleTime)** - eerste waarde afgegeven aan het begin van de periode.
- **[debounce](. /debounceTime)** - geeft een waarde na een periode van inactiviteit.
- **[sample](. /sampleTime)** - sample op de timing van een andere observable.

## Samenvatting.

De `audit` operator geeft de laatste waarde binnen een periode die dynamisch wordt bepaald door een aangepaste Observable.

- Dynamische periodecontrole is mogelijk.
- ✅ Adaptieve bemonstering gebaseerd op belasting
- ✅ Regeling op basis van andere stromen
- ⚠️ Elke keer moet een nieuwe Observable worden gegenereerd
- ⚠️ Geheugengevoelig voor frequente uitgifte
