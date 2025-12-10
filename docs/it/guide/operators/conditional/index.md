---
description: Gli operatori condizionali RxJS sono operatori per prendere decisioni condizionali sui valori in uno stream, impostare valori predefiniti e valutare condizioni. defaultIfEmpty, every, isEmpty, ecc. possono essere usati per implementare scenari pratici come elaborazione di stream vuoti, controllo di tutti i valori e verifica dell'esistenza con la type safety di TypeScript.
---

# Operatori Condizionali

Gli operatori condizionali di RxJS sono usati per **determinare o valutare condizionalmente** il valore di uno stream.
Come impostare un valore predefinito per uno stream vuoto, o controllare se tutti i valori soddisfano una condizione,
possono essere utilizzati in scenari pratici.

Questa pagina introduce ogni operatore in tre fasi: "Sintassi e Operazione Base", "Esempi di Utilizzo Tipici" e "Esempi di Codice Pratici (con UI)" nella seguente struttura.

Comprendi per quali casi d'uso ogni operatore è adatto,
e combinarli ti permetterà di progettare elaborazioni reattive più robuste e in linea con le tue intenzioni.

> [!NOTE]
> `iif` e `defer` sono **Creation Functions** (funzioni di creazione Observable) e non sono operatori condizionali. Vedi [Capitolo 3: Creation Functions](/it/guide/creation-functions/) per questi.

## Lista degli Operatori

Di seguito è riportata una lista dei principali operatori condizionali e le loro caratteristiche.

| Operatore | Descrizione |
|--------------|------|
| [defaultIfEmpty](./defaultIfEmpty.md) | Valore alternativo quando nessun valore viene emesso |
| [every](./every.md) | Valuta se tutti i valori corrispondono a una condizione |
| [isEmpty](./isEmpty.md) | Controlla se viene emesso qualche valore |

> Per **combinazioni pratiche** e **applicazioni basate su casi d'uso** degli operatori, vedi la sezione [Casi d'Uso Pratici](./practical-use-cases.md) alla fine.


## Sii Consapevole dell'Integrazione con Altre Categorie

Gli operatori condizionali sono utili solo in combinazione con altri operatori di trasformazione, combinazione e utility.
Ad esempio, è comune combinarli con `switchMap` e `catchError` per eseguire "switching API e elaborazione di recupero".

Per casi d'uso più pratici, vedi [Casi d'Uso Pratici](./practical-use-cases.md) per spiegazioni dettagliate.
