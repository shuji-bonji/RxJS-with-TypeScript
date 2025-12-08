---
description: "L'opérateur scan est un opérateur RxJS qui produit des résultats intermédiaires tout en accumulant chaque valeur séquentiellement. Contrairement à reduce(), il émet des résultats à chaque fois qu'une valeur arrive, ce qui le rend utile pour l'agrégation en temps réel, la gestion d'état, les compteurs cumulatifs et les calculs en continu. Explique l'implémentation TypeScript sécurisée."
---

# scan - Générer des valeurs de manière cumulative

L'opérateur `scan` applique une fonction d'accumulation à chaque valeur d'un flux et produit des **résultats intermédiaires séquentiels**.
Similaire à `Array.prototype.reduce`, mais différent dans le fait qu'il produit des résultats intermédiaires séquentiellement avant que toutes les valeurs n'arrivent.

## 🔰 Syntaxe de base et utilisation

```ts
import { of } from 'rxjs';
import { scan } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(scan((acc, curr) => acc + curr, 0))
  .subscribe(console.log);

// Sortie : 1, 3, 6, 10, 15
```

- `acc` est la valeur accumulée, `curr` est la valeur courante.
- Commence à partir de la valeur initiale (dans ce cas `0`) et accumule séquentiellement.

[🌐 Documentation officielle RxJS - scan](https://rxjs.dev/api/operators/scan)

## 💡 Modes d'utilisation typiques

- Comptage ou agrégation de scores
- Gestion en temps réel de l'état de validation des formulaires
- Traitement cumulatif d'événements mis en mémoire tampon
- Construction de données pour les graphiques d'agrégation en temps réel

## 🧠 Exemple de code pratique (avec interface utilisateur)

Affiche le nombre de clics cumulés à chaque fois que le bouton est cliqué.

```ts
import { fromEvent } from 'rxjs';
import { scan, tap } from 'rxjs';

// Création du bouton
const button = document.createElement('button');
button.textContent = 'Cliquer';
document.body.appendChild(button);

// Zone de sortie
const counter = document.createElement('div');
counter.style.marginTop = '10px';
document.body.appendChild(counter);

// Cumul des événements de clic
fromEvent(button, 'click')
  .pipe(
    tap((v) => console.log(v)),
    scan((count) => count + 1, 0)
  )
  .subscribe((count) => {
    counter.textContent = `Nombre de clics : ${count}`;
  });
```

- Chaque clic sur le bouton incrémente le compteur de 1.
- L'utilisation de `scan` permet d'écrire **une logique de comptage simple sans gestion d'état**.
