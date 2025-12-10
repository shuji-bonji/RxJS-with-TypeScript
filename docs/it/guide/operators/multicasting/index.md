---
description: Spiega gli operatori RxJS relativi al multicasting, incluse strategie pratiche di multicast come l'uso di share, shareReplay, publish e multicast, conversione da cold a hot, distribuzione efficiente dei valori a più subscriber e prevenzione dei memory leak. Impara pattern di implementazione per l'ottimizzazione delle performance con l'inferenza dei tipi TypeScript per la condivisione type-safe degli stream.
---

# Operatori Utilizzati nel Multicasting

RxJS fornisce diversi operatori dedicati per realizzare il "multicasting", condividendo lo stesso output Observable con più subscriber.

Questa pagina introduce brevemente gli operatori tipici relativi al multicasting dalla prospettiva **degli operatori**,
e organizza il loro utilizzo e i punti a cui prestare attenzione.

> ❗ Per spiegazioni concettuali del multicasting, spiegazioni strutturali usando Subject ed esempi di codice concreti,
> vedi [Meccanismo di Multicasting](/it/guide/subjects/multicasting).

## Principali Operatori Relativi al Multicasting

| Operatore | Caratteristiche | Note |
|--------------|------|------|
| **[share()](/it/guide/operators/multicasting/share)** | Il metodo multicast più semplice. Internamente equivalente a `publish().refCount()` | Sufficiente per molti casi d'uso |
| **[shareReplay()](/it/guide/operators/multicasting/shareReplay)** | Oltre al multicasting, fornisce valori recenti alla risottoscrizione | Quando è richiesto il riutilizzo dello stato |
| `publish()` + `refCount()` | Configurazione multicast con timing di esecuzione controllabile | Configurazione classica e flessibile |
| `multicast()` | API di basso livello che passa esplicitamente `Subject` | Utile quando vuoi usare un Subject personalizzato |

## Confronto dei Pattern di Multicasting

| Operatore | Caratteristiche | Caso d'Uso |
|------------|------|-------------|
| **[share()](/it/guide/operators/multicasting/share)** | Multicast base | Uso simultaneo tra più componenti |
| **[shareReplay(n)](/it/guide/operators/multicasting/shareReplay)** | Buffer degli ultimi n valori | Sottoscrizione tardiva/condivisione stato |
| `publish() + refCount()` | Controllo più granulare possibile | Quando serve controllo avanzato |
| `multicast(() => new Subject())` | Personalizzazione completa | Quando servono tipi di Subject speciali |

## Precauzioni nell'Uso del Multicasting

1. **Comprendere il timing**: Capire che il valore ricevuto dipende da quando inizia la sottoscrizione
2. **Gestione del lifecycle**: Specialmente usando `refCount`, lo stream viene completato quando il numero di subscriber raggiunge zero
3. **Gestione errori**: Se si verifica un errore in un Observable multicast, influenzerà tutti i subscriber
4. **Gestione memoria**: Prestare attenzione ai memory leak quando si usa `shareReplay`, ecc.
