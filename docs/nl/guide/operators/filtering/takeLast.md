---
description: "takeLast is een RxJS filteroperator die alleen de laatste N waarden uitvoert wanneer een Observable stream wordt voltooid. Het is ideaal voor situaties waarin alleen de laatste waarde van de hele stream nodig is, zoals het krijgen van de laatste telling in het log, het weergeven van de top N waarden in het leaderboard, of de laatste gegevens samenvatting bij voltooiing. Kan niet worden gebruikt met oneindige streams omdat het wordt vastgehouden in een buffer tot voltooiing."
---

# takeLast - de laatste N waarden krijgen

De `takeLast` operator voert alleen de laatste N waarden uit op het moment dat de stream **voltooid** is. Het bewaart de waarden in een buffer totdat de stream is voltooid en voert ze samen uit na voltooiing.

## 🔰 Basis syntaxis en gebruik

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0van (tot)9naar

numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Uitgang: 7, 8, 9
```

**Bewerkingsstroom**:.
1. stream geeft 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 uit
2. intern houden laatste 3 in buffer
3. stream voltooid 4. bufferwaarden 7, 8, 9
4. bufferwaarden 7, 8, 9 worden achtereenvolgens uitgevoerd

[🌐 Officiële RxJS documentatie - `takeLast`](https://rxjs.dev/api/operators/takeLast)

## 🆚 Contrast met take.

`take` en `takeLast` hebben contrasterend gedrag.

```ts
import { range } from 'rxjs';
import { take, takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0van (tot)9naar

// take: De eersteNDe eerste krijgen
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Uitgang: 0, 1, 2(onmiddellijke uitvoer)

// takeLast: De laatsteNDe eerste krijgen
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Uitgang: 7, 8, 9(wacht op voltooiing voor uitvoer)
```

TABEL 10

## 💡 Typisch gebruikspatroon

1. **Verzamel de laatste N logboekvermeldingen**.

```ts
   import { from } from 'rxjs';
   import { takeLast } from 'rxjs';

   interface LogEntry {
     timestamp: number;
     level: 'info' | 'warn' | 'error';
     message: string;
   }

   const logs$ = from([
     { timestamp: 1, level: 'info' as const, message: 'App started' },
     { timestamp: 2, level: 'info' as const, message: 'User logged in' },
     { timestamp: 3, level: 'warn' as const, message: 'Slow query detected' },
     { timestamp: 4, level: 'error' as const, message: 'Connection failed' },
     { timestamp: 5, level: 'info' as const, message: 'Retry successful' },
   ] as LogEntry[]);

   // Haal de laatste3Haal de laatste logs
   logs$.pipe(
     takeLast(3)
   ).subscribe(log => {
     console.log(`[${log.level}] ${log.message}`);
   });
   // Uitgang:
   // [warn] Slow query detected
   // [error] Connection failed
   // [info] Retry successful
   ```

2. **Top van leaderboardNDe top ophalen**
   ```ts
   import { from } from 'rxjs';
   import { takeLast } from 'rxjs';

   interface Score {
     player: string;
     score: number;
   }

   const scores$ = from([
     { player: 'Alice', score: 100 },
     { player: 'Bob', score: 150 },
     { player: 'Charlie', score: 200 },
     { player: 'Dave', score: 180 },
     { player: 'Eve', score: 220 }
   ] as Score[]).pipe(
     // Aangenomen gesorteerd op score
   );

   // Verkrijg de top3Verkrijg de
   scores$.pipe(
     takeLast(3)
   ).subscribe(score => {
     console.log(`${score.player}: ${score.score}`);
   });
   // Uitgang: Charlie: 200, Dave: 180, Eve: 220
   ```

3. **Definitieve samenvatting nadat de gegevensverwerking is voltooidNSamenvatting van gevallen**
   ```ts
   import { interval } from 'rxjs';
   import { take, map, takeLast } from 'rxjs';

   // Simulatie van sensorgegevens
   const sensorData$ = interval(100).pipe(
     take(20),
     map(i => ({
       id: i,
       temperature: 20 + Math.random() * 10
     }))
   );

   // De laatste5Berekening van de gemiddelde temperatuur van de koffer
   sensorData$.pipe(
     takeLast(5)
   ).subscribe({
     next: data => {
       console.log(`gegevens${data.id}: ${data.temperature.toFixed(1)}°C`);
     },
     complete: () => {
       console.log('Laatste5Gegevensverwerving van de koffer voltooid');
     }
   });
   ```

## 🧠 Praktisch codevoorbeeld (invoerhistoriek)

Voorbeeld van weergave van de laatste3Dit is een voorbeeld van het weergeven van de laatste door de gebruiker ingevoerde waarden.

```

ts.
import { fromEvent, Subject } from 'rxjs';
import { takeLast } from 'rxjs';

// UI-elementen maken
const container = document.createElement('div');
document.body.appendChild(container);

const input = document.createElement('input');
input.placeholder = "Voer een waarde in en Enter";
container.appendChild(input);

const submitButton = document.createElement('knop');
submitButton.textContent = "Toon geschiedenis (laatste 3)";
container.appendChild(submitButton);

const historyDisplay = document.createElement('div');
historyDisplay.style.marginTop = '10px';
container.appendChild(historyDisplay);

// Subject voor invoerwaarden
const inputs$ = nieuw Subject();.

// **IMPORTANT**: abonnement take eerst instellen
inputs$.pipe(
  takeLast(3)
).subscribe({
  next: (waarde) => {
    const item = document.createElement('div');
    item.textContent = `- ${waarde}`;
    historyDisplay.appendChild(item);
  },.
  complete: () => {
    const note = document.createElement('div');
    note.style.marginTop = '5px';
    noot.style.kleur = 'grijs';
    note.textContent = '(Herlaad de pagina om opnieuw te typen)';
    historyDisplay.appendChild(noot);

    // Invoervelden en knoppen uitschakelen
    input.disabled = true;
    submitButton.disabled = true;
  }
});

// Input toevoegen met de Enter-toets
fromEvent<KeyboardEvent>(input, 'keydown').subscribe(event => {
  if (event.key === 'Enter' && input.value.trim()) {
    inputs$.next(input.value);
    console.log(`Toevoegen: ${input.value}`);
    input.value = '';
  }
});

// Voltooien met knopklik en geschiedenis weergeven
fromEvent(submitButton, 'click').subscribe() => {
  historyDisplay.innerHTML = '<strong>Geschiedenis (laatste 3):</strong><br>';
  inputs$.complete(); // stream compleet → takeLast vuurt af
});

```

> [!IMPORTANT]
> **Belangrijke punten**:
> - `takeLast(3)` Abonneren op de**eerste.**moet eerst worden ingesteld
> - wanneer op de knop wordt geklikt. `complete()` de laatste van de tot dan toe ontvangen waarden wordt uitgevoerd.3De laatste tot dan toe ontvangen waarde wordt uitgevoerd.
> - `complete()` Na oproepen**Na het oproepen van**naar `subscribe` stromen de waarden niet.

## ⚠️ Een belangrijk punt om op te merken

> [!WARNING]
> `takeLast` is om te wachten tot de stroom**Wachten tot voltooiing**Daarom werkt het niet met oneindige streams. Ook moet de`takeLast(n)` van dengroot is, verbruikt het veel geheugen.

### 1. Kan niet gebruikt worden met oneindige streams.

`takeLast` werkt niet met oneindige streams omdat het wacht tot de stream voltooid is.

```

ts.
import { interval } from 'rxjs';
import { takeLast } from 'rxjs';

// ❌ Slecht voorbeeld: takeLast gebruiken met oneindige streams
interval(1000).pipe(
  takeLast(3)
).subscribe(console.log);.
// Er wordt niets uitgevoerd (omdat de stream nooit wordt voltooid)

```

**Oplossing.**: `take` Gebruik een eindige stream in combinatie met

```

ts.
import { interval } from 'rxjs';
import { take, takeLast } from 'rxjs';

// ✅ Goed voorbeeld: eindige stroom gebruik dan takeLast
interval(1000).pipe(
  take(10), // compleet met de eerste 10
  takeLast(3) // neem de laatste 3 mee
).subscribe(console.log);.
// Uitvoer: 7, 8, 9

```

### 2. Let op het geheugengebruik

`takeLast(n)` werkt niet met eindige streams omdat het het laatstenstuk vasthoudt in de buffer,ngroot is, verbruikt het meer geheugen.

```

ts.
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

// ⚠️ Opmerking: grote hoeveelheden gegevens worden bewaard in een buffer
range(0, 1000000).pipe(
  takeLast(100000) // 100.000 records bewaard in het geheugen
).subscribe(console.log);.

```

## 🎯 last Het verschil tussen

```

ts.
import { range } from 'rxjs';
import { last, takeLast } from 'rxjs';

const numbers$ = range(0, 10);

// last: alleen de laatste
getallen$.pipe(
  last()
).subscribe(console.log);
// uitvoer: 9

// takeLast(1): laatste (uitvoer als enkele waarde, niet array)
numbers$.pipe(
  takeLast(1)
).subscribe(console.log);.
// Uitvoer: 9

// takeLast(3): laatste 3
getallen$.pipe(
  takeLast(3)
).subscribe(console.log);
// Uitvoer: 7, 8, 9

```

| operator | Aantal overnames | Specificatie voorwaarde | Gebruik |
|---|---|---|---|
| `last()` | 1Aantal | Mogelijk | De laatste1Stukken of het laatste stuk dat aan de voorwaarde voldoet1Aantal |
| `takeLast(n)` | nAantal | Onmogelijk | De laatstenVerkrijg gewoon het laatste stuk dat aan de voorwaarde voldoet |

## 📋 Type-veilig gebruik

TypeScript Dit is een voorbeeld van een typeveilige implementatie die gebruik maakt van generieken in

```

ts.
import { Observable, from } from 'rxjs';
import { takeLast } from 'rxjs';

interface Transactie {
  id: string;
  bedrag: getal;
  tijdstempel: datum;
  status: 'pending' | 'complete' | 'failed'; }
}

functie getRecentTransactions(
  transacties$: Observable,.
  count: getal
): Observable {
  return transacties$.pipe(
    takeLast(count)
  );
}

// Gebruiksvoorbeeld
const transacties$ = from([.
  { id: '1', bedrag: 100, tijdstempel: nieuwe Datum('2025-01-01'), status: 'compleet' als const }.
  { id: '2', bedrag: 200, tijdstempel: nieuwe Datum('2025-01-02'), status: 'compleet' als const }
  { id: '3', amount: 150, timestamp: new Date('2025-01-03'), status: 'pending' as const }
  { id: '4', amount: 300, timestamp: new Date('2025-01-04'), status: 'complete' as const }
  { id: '5', bedrag: 250, tijdstempel: nieuwe Datum('2025-01-05'), status: 'mislukt' als const }
] als Transactie[]);.

// Verkrijg de drie meest recente transacties
getRecentTransactions(transactions$, 3).subscribe(tx => {
  console.log(`${tx.id}: ${tx.amount} yen (${tx.status})`);
});
// Uitvoer:.
// 3: 150 yen (in behandeling)
// 4: 300 yen (compleet)
// 5: 250 yen (mislukt)

```

## 🔄 skip en takeLast combinatie van

Het middelste deel van de waarde wordt uitgesloten en alleen de laatsteNAlleen de laatste kan worden opgehaald.

```

ts
import { range } from 'rxjs';
import { skip, takeLast } from 'rxjs';

const nummers$ = range(0, 10); // 0 tot 9

// sla de eerste 5 over en neem de resterende laatste 3
getallen$.pipe(
  skip(5), // skip 0, 1, 2, 3, 4
  takeLast(3) // neem de laatste 3 van de resterende 5, 6, 7, 8, 9
).subscribe(console.log);.
// uitvoer: 7, 8, 9
```

## Samenvatting

### Wanneer takeLast moet worden gebruikt.
- ✅ Als je de laatste N gegevens in een stroom nodig hebt
- ✅ Als je de laatste N logs of transacties wilt krijgen
- ✅ Als de stream gegarandeerd wordt voltooid
- ✅ Als je een samenvatting of top N records van gegevens wilt weergeven

### Wanneer je take moet gebruiken.
- ✅ Als je de eerste N gegevens in de stream nodig hebt
- ✅ Als je de resultaten onmiddellijk wilt krijgen
- ✅ Als je een deel van een oneindige stroom wilt krijgen

### Opmerkingen.
- ⚠️ Kan niet gebruikt worden met oneindige stromen (omdat ze niet compleet zijn)
- ⚠️ Grote n in `takeLast(n)` verbruikt geheugen
- ⚠️ Uitvoer wordt gecompileerd na voltooiing (niet onmiddellijk)
- ⚠️ Moet vaak gecombineerd worden met `take(n)` om een eindige stroom te maken

## Volgende stap.

- **[take](. /take)** - leer hoe je de eerste n waarden krijgt.
- **[last](. /last)** - leer hoe je de laatste 1 waarde krijgt.
- **[skip](. /skip)** - leer de eerste N waarden over te slaan
- Filter](. /filter)** - leer filteren op basis van voorwaarden
- **[filtering-operator-praktische-gebruiksgevallen](. /practical-use-cases)** - leer hoe u echte use-cases kunt gebruiken
