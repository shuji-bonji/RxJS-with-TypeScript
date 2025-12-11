---
description: "Observable is het kernbegrip van RxJS en vertegenwoordigt een datastroom die zich in de tijd ontwikkelt. Een gedetailleerde uitleg van het verschil met Promise, het mechanisme van abonneren (subscribe) en afmelden (unsubscribe), Cold en Hot Observable, en type-definities in TypeScript."
---

# Wat is Observable

[📘 RxJS Officieel: Observable](https://rxjs.dev/api/index/class/Observable)

Een Observable in RxJS is een kerncomponent die "de stroom van gegevens die zich in de loop van de tijd ontwikkelt (stream)" vertegenwoordigt. Het is ontworpen op basis van het Observer-patroon en maakt het mogelijk om asynchrone verwerking en event-driven processing op uniforme wijze te behandelen.

## Rol van Observable

Observable functioneert als een "gegevensproducent" die meerdere waarden in de loop van de tijd uitgeeft. Daartegenover staat de Observer als "consument", die waarden ontvangt via `subscribe()`.

In het volgende voorbeeld maken we een **Observable (producent)** genaamd `observable$`, en een **Observer (consument)** die abonneert om waarden te ontvangen.

```ts
import { Observable } from 'rxjs';

// Observable (producent) aanmaken
const observable$ = new Observable<number>(subscriber => {
  // Logica die wordt uitgevoerd bij abonnement
  subscriber.next(1);
  subscriber.next(2);
  subscriber.complete();
});

// Observer (consument) abonneert
observable$.subscribe({
  next: value => console.log('Volgende waarde:', value),
  error: err => console.error('Fout:', err),
  complete: () => console.log('Voltooid')
});

// Output:
// Volgende waarde: 1
// Volgende waarde: 2
// Voltooid
```

> [!NOTE]
> De functie die als argument wordt doorgegeven aan `new Observable(functie)` definieert **de logica die wordt uitgevoerd wanneer de Observable wordt geabonneerd**. Deze functie zelf is niet de producent, de gehele Observable is de producent.

## Types van notificaties

Observable stuurt de volgende 3 types notificaties naar Observer.

- `next`: Notificatie van waarden
- `error`: Notificatie bij fout (er volgen geen verdere notificaties)
- `complete`: Notificatie van normale afronding

Voor details, zie de sectie [Observer (オブザーバー) in "Lifecycle van Observable"](./observable-lifecycle.md#_2-observer-オブザーバー).

## Verschil tussen Observable en Promise

| Kenmerk | Observable | Promise |
|---|---|---|
| Meerdere waarden | ◯ | ×(Slechts één) |
| Annuleerbaar | ◯(`unsubscribe()`) | × |
| Uitgestelde uitvoering | ◯ | ◯ |
| Synchroon/Asynchroon | Beide | Alleen asynchroon |

Het grootste verschil tussen Observable en Promise is "of het meerdere waarden kan behandelen" en "of het halverwege kan worden geannuleerd".
Promise is geschikt voor eenmalige asynchrone verwerking, maar Observable heeft sterke punten bij "continu voorkomende asynchrone gegevens" zoals event streams.

Bovendien kan Observable het abonnement halverwege annuleren met `unsubscribe()`, wat ook belangrijk is vanuit het oogpunt van resource management, zoals het voorkomen van geheugen leaks en het stoppen van onnodige communicatie.

Aan de andere kant wordt Promise breed toegepast in standaard API's en maakt het intuïtief schrijven mogelijk in combinatie met `async/await`. Het is wenselijk om ze af te wisselen afhankelijk van het gebruiksdoel.

## Onderscheid tussen Cold en Hot

RxJS Observables hebben twee types: "Cold" en "Hot".

- **Cold Observable**: Elke abonnee heeft zijn eigen gegevensstroom, en de uitvoering begint wanneer er wordt geabonneerd. (Voorbeelden: `of()`, `from()`, `fromEvent()`, `ajax()`)
- **Hot Observable**: Abonnees delen dezelfde gegevensstroom, en gegevens blijven stromen ongeacht of er abonnees zijn. (Voorbeelden: `Subject`, Observable gemaakt met `share()` multicast)

Dit onderscheid heeft grote invloed op het delen van gegevens en resource-efficiëntie.
Voor details, zie de sectie ["Cold Observable en Hot Observable"](./cold-and-hot-observables.md).

## Observable en Pipeline

De ware kracht van Observable komt tot uiting wanneer je het combineert met operators via de `pipe()` methode.

```ts
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5);
numbers$.pipe(
  filter(n => n % 2 === 0), // Alleen even getallen doorlaten
  map(n => n * 10)          // Vermenigvuldigen met 10
).subscribe(value => console.log(value));
// Output: 20, 40
```

## Lifecycle van Observable

Observable heeft de volgende lifecycle:

1. **Creatie** - Genereren van Observable instantie
2. **Abonnement** - Start van gegevensontvangst via `subscribe()`
3. **Uitvoering** - Uitgeven van gegevens (`next`), fout (`error`), of voltooiing (`complete`)
4. **Afmelding** - Beëindigen van abonnement via `unsubscribe()`

Om resource leaks te voorkomen, is het belangrijk om onnodige Observable-abonnementen af te melden.
Voor details, zie de sectie ["Lifecycle van Observable"](./observable-lifecycle.md).

## Wanneer Observable gebruiken

- UI events (klik, scroll, toetsenbord operaties, etc.)
- HTTP requests
- Op tijd gebaseerde verwerking (intervallen of timers)
- WebSocket of realtime communicatie
- Applicatie status management

## Samenvatting

Observable is de basis voor het flexibel en uniform behandelen van asynchrone gegevens. Als kernbegrip van ReactiveX (RxJS) kan het complexe asynchrone verwerking en event streams bondig uitdrukken.
