---
description: "raceWith is een RxJS join operator die uit de oorspronkelijke Observable en andere Observables alleen de stream overneemt die de waarde als eerste heeft uitgegeven. Het is ideaal voor time-out implementaties, fallback processen, snelste overname van meerdere gegevensbronnen en andere situaties waarin selectie op basis van race condities vereist is; de pipe operator versie is handig voor gebruik in pijplijnen."
---

# raceWith - de snelste stroom overnemen

De `raceWith` operator **adopteert** alleen de eerste Observable die een waarde afgeeft uit de oorspronkelijke Observable en de andere gespecificeerde Observables,
negeert daarna de andere Observable.
Dit is de Pipeable Operator versie van de `race` van Creation Function.

## 🔰 Basissyntaxis en gebruik

```ts
import { timer } from 'rxjs';
import { raceWith, map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Langzaam. (5Seconden)'));
const medium$ = timer(3000).pipe(map(() => 'Normaal (3Seconden)'));
const fast$ = timer(2000).pipe(map(() => 'Snel (2Seconden)'));

slow$
  .pipe(raceWith(medium$, fast$))
  .subscribe(console.log);

// Uitvoer: Snel (2Seconden)
```

- Alleen de Observable die als eerste de waarde heeft afgegeven (`fast$` in dit voorbeeld) is de winnaar en gaat verder met volgende streams.
- Andere Observable worden genegeerd.

[🌐 Officiële RxJS documentatie - `raceWith`](https://rxjs.dev/api/operators/raceWith)

## 💡 Typisch gebruikspatroon.

- **Time-out implementatie**: race de time-out timer tegen het hoofdproces.
- **Fallback-verwerking**: kies de snelste uit meerdere gegevensbronnen
- **Gebruikersinteractie**: neem de snelste van klik en auto-progress

## Praktische codevoorbeelden (met UI)

Dit is een voorbeeld van een race tussen handmatig klikken en automatische voortgangstimer en het overnemen van de snelste.

```ts
import { fromEvent, timer } from 'rxjs';
import { raceWith, map, take } from 'rxjs';

// Uitvoergedeelte maken
const output = document.createElement('div');
output.innerHTML = '<h3>raceWith Praktische voorbeelden van:</h3>';
document.body.appendChild(output);

// Knoppen maken
const button = document.createElement('button');
button.textContent = 'Ga handmatig te werk (5Klik binnen enkele seconden)';
document.body.appendChild(button);

// Wachtbericht
const waiting = document.createElement('div');
waiting.textContent = '5Klik binnen enkele seconden op de knop of wacht op automatische voortgang...';
waiting.style.marginTop = '10px';
output.appendChild(waiting);

// Stroom voor handmatig klikken
const manualClick$ = fromEvent(button, 'click').pipe(
  take(1),
  map(() => '👆 Handmatig klikken is geselecteerd！')
);

// Automatische voortgangstimer (5(na 2 seconden)
const autoProgress$ = timer(5000).pipe(
  map(() => '⏰ Automatische voortgang is geselecteerd！')
);

// Race-uitvoering
manualClick$
  .pipe(raceWith(autoProgress$))
  .subscribe((winner) => {
    waiting.remove();
    button.disabled = true;

    const result = document.createElement('div');
    result.innerHTML = `<strong>${winner}</strong>`;
    result.style.color = 'green';
    result.style.fontSize = '18px';
    result.style.marginTop = '10px';
    output.appendChild(result);
  });
```

- Handmatig klikken wordt aangenomen als er binnen 5 seconden op de knop wordt geklikt.
- Na 5 seconden wordt de automatische voortgang aangenomen.
- De snelste is de winnaar**, de langzamere wordt genegeerd.

## Verschillen met Creation Function `race`.

### Basisverschillen.

```ts
import { timer } from 'rxjs';
import { raceWith, map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Langzaam. (5Seconden)'));
const medium$ = timer(3000).pipe(map(() => 'Normaal (3Seconden)'));
const fast$ = timer(2000).pipe(map(() => 'Snel (2Seconden)'));

slow$
  .pipe(raceWith(medium$, fast$))
  .subscribe(console.log);

// Uitvoer: Snel (2Seconden)
```

### Specifieke voorbeelden van gebruik

**Als je alleen een eenvoudige race wilt, is Creation Function de beste manier**.


```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

const server1$ = timer(3000).pipe(map(() => 'Server1Antwoord van'));
const server2$ = timer(2000).pipe(map(() => 'Server2Antwoord van'));
const server3$ = timer(4000).pipe(map(() => 'Server3Antwoord van'));

// Eenvoudig en gemakkelijk te lezen
race(server1$, server2$, server3$).subscribe(response => {
  console.log('Aangenomen:', response);
});
// Uitvoer: Aangenomen: Server2Antwoord van (Snelste2Seconden)
```

**Als je een conversieproces wilt toevoegen aan de hoofdstroom, wordt de Pipeable Operator aanbevolen**.

```ts
import { fromEvent, timer, of } from 'rxjs';
import { raceWith, map, switchMap, catchError } from 'rxjs';

const searchButton = document.createElement('button');
searchButton.textContent = 'Zoeken';
document.body.appendChild(searchButton);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Mainstream: Zoekopdrachten van gebruikers
const userSearch$ = fromEvent(searchButton, 'click').pipe(
  switchMap(() => {
    output.textContent = 'Zoeken...';

    // APIEen gesprek simuleren (3(duurt seconden)
    return timer(3000).pipe(
      map(() => '🔍 Zoekresultaten: 100Hits'),
      catchError((err: unknown) => of('❌ Fout opgetreden'))
    );
  })
);

// ✅ Pipeable OperatorEditie - Voltooid in één pijplijn
userSearch$
  .pipe(
    raceWith(
      // Time-out (in2seconden)
      timer(2000).pipe(
        map(() => '⏱️ Time-out (s): Zoeken duurt te lang')
      )
    )
  )
  .subscribe(result => {
    output.textContent = result;
  });

// ❌ Creation FunctionEditie - Mainstream moet apart worden geschreven
import { race } from 'rxjs';
race(
  userSearch$,
  timer(2000).pipe(
    map(() => '⏱️ Time-out (s): Zoeken duurt te lang')
  )
).subscribe(result => {
  output.textContent = result;
});
```

**Implementatie van fallbackverwerking**

```ts
import { timer, throwError } from 'rxjs';
import { raceWith, map, mergeMap, catchError, delay } from 'rxjs';
import { of } from 'rxjs';

// UIAanmaak
const output = document.createElement('div');
output.innerHTML = '<h3>Gegevensverwerving (met fall-back)</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Begin gegevensverzameling';
document.body.appendChild(button);

const statusArea = document.createElement('div');
statusArea.style.marginTop = '10px';
output.appendChild(statusArea);

button.addEventListener('click', () => {
  statusArea.textContent = 'Tijdens overname...';

  // HoofdAPI(Prioriteit)：Tijd々Storing
  const mainApi$ = timer(1500).pipe(
    mergeMap(() => {
      const success = Math.random() > 0.5;
      if (success) {
        return of('✅ HoofdAPISuccesvolle overname van');
      } else {
        return throwError(() => new Error('HoofdAPIMislukking'));
      }
    }),
    catchError((err: unknown) => {
      console.log('HoofdAPIFout, overgedragen aan noodoplossing...');
      // Vertraging bij fout, overdragen aan noodoplossing
      return of('').pipe(delay(10000));
    })
  );

  // ✅ Pipeable OperatorEditie - HoofdAPITerugval toevoegen aan
  mainApi$
    .pipe(
      raceWith(
        // TerugvalAPI(Terugval)：Iets langzamer maar betrouwbaar
        timer(2000).pipe(
          map(() => '🔄 TerugvalAPITeruggehaald van')
        )
      )
    )
    .subscribe(result => {
      if (result) {
        statusArea.textContent = result;
        statusArea.style.color = result.includes('Hoofd') ? 'green' : 'orange';
      }
    });
});
```

**Haal de snelste uit meerdere gegevensbronnen**.

```ts
import { timer, fromEvent } from 'rxjs';
import { raceWith, map, mergeMap } from 'rxjs';

const output = document.createElement('div');
output.innerHTML = '<h3>MeerdereCDNSnelste laden van</h3>';
document.body.appendChild(output);

const loadButton = document.createElement('button');
loadButton.textContent = 'Bibliotheek laden';
document.body.appendChild(loadButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
output.appendChild(result);

fromEvent(loadButton, 'click').pipe(
  mergeMap(() => {
    result.textContent = 'Laden...';

    // CDN1Laden (gesimuleerd) van
    const cdn1$ = timer(Math.random() * 3000).pipe(
      map(() => ({ source: 'CDN1 (US)', data: 'library.js' }))
    );

    // ✅ Pipeable OperatorEditie - CDN1in de hoofd- en andereCDNals concurrenten.
    return cdn1$.pipe(
      raceWith(
        // CDN2Laden (gesimuleerd) van
        timer(Math.random() * 3000).pipe(
          map(() => ({ source: 'CDN2 (EU)', data: 'library.js' }))
        ),
        // CDN3Laden (gesimuleerd) van
        timer(Math.random() * 3000).pipe(
          map(() => ({ source: 'CDN3 (Asia)', data: 'library.js' }))
        )
      )
    );
  })
).subscribe(response => {
  result.innerHTML = `
    <strong>✅ Bezig met laden</strong><br>
    Verworven van: ${response.source}<br>
    Bestand: ${response.data}
  `;
  result.style.color = 'green';
});
```

### Samenvatting.

- **`race`**: ideaal als u gewoon de snelste uit meerdere streams wilt overnemen
- raceWith: ideaal als u time-outs en fallbacks wilt implementeren tijdens het transformeren en verwerken van de hoofdstroom.

## ⚠️ Opmerkingen.

### Voorbeeld van time-outimplementatie

Time-out afhandeling implementeren met `raceWith`.

```ts
import { of, timer, throwError } from 'rxjs';
import { raceWith, delay, mergeMap } from 'rxjs';

// Tijdrovend proces (3seconden)
const slowRequest$ = of('Succesvolle gegevensverwerving').pipe(delay(3000));

// Time-out (in2seconden)
const timeout$ = timer(2000).pipe(
  mergeMap(() => throwError(() => new Error('Time-out (s)')))
);

slowRequest$
  .pipe(raceWith(timeout$))
  .subscribe({
    next: console.log,
    error: err => console.error(err.message)
  });
// Uitvoer: Time-out (s)
```

### Alle streams zijn geabonneerd op

raceWith` abonneert alle Observable totdat er een winnaar is.
Nadat de winnaar is bepaald, wordt de verliezende Observable automatisch uitgeschreven.

```ts
import { timer } from 'rxjs';
import { raceWith, tap, map } from 'rxjs';

const slow$ = timer(3000).pipe(
  tap(() => console.log('slow$ Vuren')),
  map(() => 'slow')
);

const fast$ = timer(1000).pipe(
  tap(() => console.log('fast$ Vuren')),
  map(() => 'fast')
);

slow$.pipe(raceWith(fast$)).subscribe(console.log);
// Uitvoer:
// fast$ Vuren
// fast
// (slow$is1Uitgeschreven bij seconde,3niet afgevuurd aan het einde van de tweede periode.)
```

### Voor synchrone Observable.

Als alles synchroon wordt uitgegeven, is de eerst geregistreerde de winnaar.

```ts
import { of } from 'rxjs';
import { raceWith } from 'rxjs';

of('A').pipe(
  raceWith(of('B'), of('C'))
).subscribe(console.log);
// Uitvoer: A (omdat het eerst was ingeschreven op)
```

## 📚 Verwante operatoren.

- **[race](/nl/guide/creation-functions/selection/race)** - Creation Function-versie.
- **[timeout](/nl/guide/operators/utility/timeout)** - Timeout-operator.
- **[mergeWith](/nl/guide/operators/combination/mergeWith)** - Alle stromen samenvoegen.
