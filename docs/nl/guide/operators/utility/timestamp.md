---
description: De timestamp operator voegt een tijdstempel toe aan elke waarde en registreert de tijd dat de waarde werd uitgezonden, wat kan worden gebruikt voor prestatiemeting en debugging.
---

# timestamp - Voeg tijdstempel toe

De `timestamp` operator **voegt een tijdstempel toe** aan elke waarde in de stream. Dit kan worden gebruikt voor prestatiemeting, debugging en tijdreeksanalyse van gebeurtenissen door de exacte tijd te registreren dat de waarde werd uitgezonden.

## 🔰 Basissyntax en werking

Converteert elke waarde naar een object met tijdstempel.

```ts
import { interval } from 'rxjs';
import { timestamp, take } from 'rxjs';

interval(1000)
  .pipe(
    take(3),
    timestamp()
  )
  .subscribe(console.log);

// Uitvoer:
// { value: 0, timestamp: 1640000000000 }
// { value: 1, timestamp: 1640000001000 }
// { value: 2, timestamp: 1640000002000 }
```

Het geretourneerde object heeft de volgende structuur:
- `value`: De originele waarde
- `timestamp`: Tijdstempel (Unix-tijd in milliseconden)

[🌐 RxJS Officiële Documentatie - timestamp](https://rxjs.dev/api/index/function/timestamp)

## 💡 Typische gebruiksvoorbeelden

- **Prestatiemeting**: Meet verwerkingstijd
- **Gebeurtenistiming analyse**: Meet intervallen tussen gebruikersacties
- **Debugging en logging**: Registreren van de timing van waarde-uitgifte
- **Tijdreeksdata registratie**: Tijdgestempelde opslag van sensordata, etc.

## 🧪 Praktisch codevoorbeeld 1: Meten van klikintervallen

Dit is een voorbeeld van het meten van het klikinterval van de gebruiker.

```ts
import { fromEvent } from 'rxjs';
import { timestamp, pairwise, map } from 'rxjs';

// UI creatie
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'timestamp - Klikinterval meting';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'Klik alstublieft';
button.style.marginBottom = '10px';
button.style.padding = '10px 20px';
button.style.fontSize = '16px';
container.appendChild(button);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '250px';
output.style.overflow = 'auto';
container.appendChild(output);

let clickCount = 0;

function addLog(message: string, color: string = '#e3f2fd') {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;
  logItem.textContent = message;
  output.insertBefore(logItem, output.firstChild);  // Toon nieuwste bovenaan
}

fromEvent(button, 'click')
  .pipe(
    timestamp(),
    pairwise(),
    map(([prev, curr]) => {
      const interval = curr.timestamp - prev.timestamp;
      return {
        clickNumber: clickCount + 1,
        interval: interval,
        timestamp: new Date(curr.timestamp).toLocaleTimeString('nl-NL')
      };
    })
  )
  .subscribe(data => {
    clickCount++;
    const color = data.interval < 500 ? '#ffcdd2' :
                  data.interval < 1000 ? '#fff9c4' : '#c8e6c9';

    const speed = data.interval < 500 ? 'Snelle klik!' :
                  data.interval < 1000 ? 'Normaal' : 'Langzaam';

    addLog(
      `${data.clickNumber}e klik: ${data.interval}ms interval [${speed}] (${data.timestamp})`,
      color
    );
  });

addLog('Klik alstublieft op de knop (interval gemeten vanaf 2e klik)', '#e3f2fd');
```

- Nauwkeurige meting van klikinterval
- Kleurgecodeerde weergave volgens snelheid
- Registreert tijd van voorkomen met tijdstempel

## 🧪 Praktisch codevoorbeeld 2: Meten van verwerkingstijd

Dit is een voorbeeld van het meten van de tijd die nodig is voor elke verwerking.

```ts
import { interval } from 'rxjs';
import { timestamp, map, take } from 'rxjs';

// UI creatie
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'timestamp - Verwerkingstijd meting';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
container2.appendChild(output2);

function addLog2(message: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '3px';
  logItem.style.fontSize = '12px';
  logItem.style.fontFamily = 'monospace';
  logItem.textContent = message;
  output2.appendChild(logItem);
}

addLog2('Verwerking gestart...');

interval(500)
  .pipe(
    take(5),
    timestamp(),  // Tijdstempel voor verwerking
    map(data => {
      const start = data.timestamp;

      // Simuleer zware verwerking (willekeurige verwerkingstijd)
      const iterations = Math.floor(Math.random() * 5000000) + 1000000;
      let sum = 0;
      for (let i = 0; i < iterations; i++) {
        sum += i;
      }

      const end = Date.now();
      const duration = end - start;

      return {
        value: data.value,
        startTime: new Date(start).toLocaleTimeString('nl-NL', { hour12: false }) +
                   '.' + (start % 1000).toString().padStart(3, '0'),
        duration: duration
      };
    })
  )
  .subscribe({
    next: result => {
      addLog2(
        `Waarde${result.value}: start=${result.startTime}, verwerkingstijd=${result.duration}ms`
      );
    },
    complete: () => {
      addLog2('--- Alle verwerking voltooid ---');
    }
  });
```

- Registreer de starttijd van elke waarde
- Meet de tijd die nodig is voor verwerking
- Gebruik voor prestatieanalyse

## Tijdstempels gebruiken

```ts
import { of } from 'rxjs';
import { timestamp, map } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    timestamp(),
    map(data => {
      // Verwerking met tijdstempel
      const date = new Date(data.timestamp);
      return {
        value: data.value,
        time: date.toISOString(),
        unixTime: data.timestamp
      };
    })
  )
  .subscribe(console.log);
// Uitvoer:
// { value: 'A', time: '2024-01-01T00:00:00.000Z', unixTime: 1704067200000 }
// ...
```

## ⚠️ Belangrijke opmerkingen

### 1. Tijdstempel precisie

Omdat JavaScript's `Date.now()` wordt gebruikt, is de precisie in milliseconden.

```ts
import { interval } from 'rxjs';
import { timestamp, take } from 'rxjs';

// Hoogfrequente gebeurtenissen (1ms interval)
interval(1)
  .pipe(
    take(3),
    timestamp()
  )
  .subscribe(data => {
    console.log(`Waarde: ${data.value}, Tijdstempel: ${data.timestamp}`);
  });
// Kan dezelfde tijdstempel hebben
```

Als u hogere precisie nodig heeft, overweeg dan `performance.now()` te gebruiken.

### 2. Tijdstempel is op uitgiftetijd

De tijdstempel is de tijd dat de waarde werd uitgezonden, niet wanneer het werd gegenereerd.

```ts
import { of, delay, timestamp } from 'rxjs';

of(1, 2, 3)
  .pipe(
    delay(1000),      // 1 seconde vertraging
    timestamp()       // Tijdstempel na vertraging
  )
  .subscribe(console.log);
```

### 3. Objectstructuur wijziging

Het gebruik van `timestamp` wikkelt de waarde in een object.

```ts
import { of, timestamp, map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    timestamp(),
    map(data => data.value * 2)  // Toegang tot originele waarde met .value
  )
  .subscribe(console.log);
// Uitvoer: 2, 4, 6
```

## 📚 Gerelateerde operators

- **[tap](./tap)** - Voer bijwerkingen uit (voor debugging)
- **[delay](./delay)** - Vaste tijdvertraging
- **[timeout](./timeout)** - Timeout-controle

## ✅ Samenvatting

De `timestamp` operator geeft een tijdstempel voor elke waarde.

- ✅ Registreert nauwkeurig de tijd dat elke waarde wordt uitgezonden
- ✅ Nuttig voor prestatiemeting
- ✅ Maakt analyse van gebeurtenisintervallen mogelijk
- ✅ Nuttig voor debugging en logging
- ⚠️ Precisie in milliseconden
- ⚠️ Waarden worden gewikkeld in objecten
- ⚠️ Tijdstempels zijn op moment van uitgifte
