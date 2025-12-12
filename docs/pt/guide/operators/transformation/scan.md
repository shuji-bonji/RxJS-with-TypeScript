---
description: O operador scan é um operador RxJS que gera resultados intermediários enquanto acumula cada valor sequencialmente, e é usado para agregação em tempo real e gerenciamento de estado.
---

# scan - Gera Valores Cumulativamente

O operador `scan` aplica uma função cumulativa a cada valor no stream e gera **resultados intermediários sequenciais**.
Semelhante a `Array.prototype.reduce` para arrays, exceto que o resultado intermediário é gerado sequencialmente antes que todos os valores sejam alcançados.

## 🔰 Sintaxe Básica e Uso

```ts
import { of } from 'rxjs';
import { scan } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(scan((acc, curr) => acc + curr, 0))
  .subscribe(console.log);

// Saída: 1, 3, 6, 10, 15

```

- `acc` é o valor acumulado, `curr` é o valor atual.
- Começa com um valor inicial (`0` neste caso) e acumula sequencialmente.

[🌐 Documentação Oficial RxJS - `scan`](https://rxjs.dev/api/operators/scan)

## 💡 Padrões Típicos de Uso

- Contagem progressiva e agregação de pontuação
- Gerenciamento de status de validação de formulário em tempo real
- Processamento cumulativo de eventos em buffer
- Construção de dados para gráficos agregados em tempo real

## 🧠 Exemplo de Código Prático (com UI)

Exibe o número cumulativo de cliques cada vez que um botão é clicado.

```ts
import { fromEvent } from 'rxjs';
import { scan, tap } from 'rxjs';

// Criar botão
const button = document.createElement('button');
button.textContent = 'Clicar';
document.body.appendChild(button);

// Criar área de saída
const counter = document.createElement('div');
counter.style.marginTop = '10px';
document.body.appendChild(counter);

// Acumular eventos de clique
fromEvent(button, 'click')
  .pipe(
    tap((v) => console.log(v)),
    scan((count) => count + 1, 0)
  )
  .subscribe((count) => {
    counter.textContent = `Contagem de cliques: ${count}`;
  });
```

- Cada vez que um botão é clicado, o contador é incrementado em 1.
- Ao usar `scan`, você pode escrever **lógica de contagem simples sem gerenciamento de estado**.
